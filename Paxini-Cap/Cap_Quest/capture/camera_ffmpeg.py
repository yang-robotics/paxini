"""Timestamped ffmpeg capture for the synchronized recorder.

ffmpeg reads ``/dev/videoN`` through V4L2 and stamps each frame with the host
wall clock (``-use_wallclock_as_timestamps 1``), so the pts in the container
*is* Unix epoch nanoseconds.  The camera must be attached to the Linux host
directly: USB/IP forwarding drops isochronous packets and collapses 720p30 to
about 5 fps.

OpenCV ``read()`` only reports when the call returned; the frame may have waited
in the driver queue for up to one frame period.  We therefore keep ffmpeg's own
container timestamps rather than re-stamping after the fact.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

# Any sane wall-clock stamp is far above this; a pts here means the capture
# fell back to a relative time base and must not be treated as epoch time.
WALLCLOCK_MIN_NS = 1_500_000_000_000
VIDEO_CODECS = ("copy", "h264", "hevc", "mjpeg")
STOP_TIMEOUT_S = 4.0

# Live preview: a second ffmpeg output writes small JPEGs to stdout.  Kept at a
# few fps and a few hundred px on purpose — this rides along with the recording
# and must not compete for encoder time (the mkv output is the copy encoder, so
# the cost here is the scaler plus mjpeg).
PREVIEW_FPS = 5.0
PREVIEW_WIDTH = 360
PREVIEW_QUALITY = 7
PREVIEW_MAX_BUFFER = 4 * 1024 * 1024

# ffmpeg runs in its own session so only this program's SIGINT reaches it. On a
# forced exit that session would be orphaned, so keep handles here and ask them
# to finalise before the process disappears.
RUNNING_PROCESSES: list[subprocess.Popen[bytes]] = []


def interrupt_running_cameras() -> None:
    """Best-effort shutdown for cameras that are still recording.

    ffmpeg runs in its own session, so only our SIGINT reaches it; that is what
    makes it finalise the mkv instead of leaving a truncated container.
    """
    for process in list(RUNNING_PROCESSES):
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except (ProcessLookupError, OSError, ValueError):
                pass
        if process in RUNNING_PROCESSES:
            RUNNING_PROCESSES.remove(process)


def log(message: str) -> None:
    print(message, flush=True)


def warn(message: str) -> None:
    print(f"[警告] {message}", file=sys.stderr, flush=True)


def discover_video_nodes(
    sysfs_root: Path = Path("/sys/class/video4linux"),
    dev_root: Path = Path("/dev"),
) -> list[str]:
    """Find one capture node per UVC device.

    A UVC camera exposes a metadata node next to its capture node; only
    ``index == 0`` is a frame source. Nodes are de-duplicated by the real path of
    their parent device so a device that exposes several index-0 nodes still
    yields a single entry.
    """
    if not sysfs_root.is_dir():
        return []

    devices: dict[str, str] = {}
    for entry in sorted(sysfs_root.glob("video*")):
        try:
            index = int((entry / "index").read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            continue
        if index != 0:
            continue
        try:
            device_key = os.path.realpath(entry / "device")
        except OSError:
            device_key = str(entry / "device")
        devices[device_key] = str(dev_root / entry.name)
    return [devices[key] for key in sorted(devices)]


def find_ffmpeg(argument: str | None = None) -> str:
    """Resolve the ffmpeg binary; ``--ffmpeg`` beats ``$FFMPEG`` beats ``PATH``."""
    requested = argument or os.environ.get("FFMPEG") or "ffmpeg"
    path = Path(requested).expanduser()
    if path.is_absolute() or path.parent != Path("."):
        if path.is_file() and os.access(path, os.X_OK):
            return str(path)
        raise RuntimeError(f"ffmpeg 不存在或不可执行：{path}")
    found = shutil.which(requested)
    if not found:
        raise RuntimeError(
            "找不到 ffmpeg（需要 5.0+，带 ffprobe，且支持 v4l2 与 -fps_mode）。"
            "装到 PATH 里即可，或用 --ffmpeg 指定路径、FFMPEG 环境变量。"
        )
    return found


def find_ffprobe(ffmpeg: str) -> str:
    sibling = Path(ffmpeg).with_name("ffprobe")
    if sibling.is_file() and os.access(sibling, os.X_OK):
        return str(sibling)
    found = shutil.which("ffprobe")
    if not found:
        raise RuntimeError(
            "找不到 ffprobe。请把它和 ffmpeg 放在同一目录，或加入 PATH。"
        )
    return found


def ffmpeg_version(ffmpeg: str) -> str:
    """Return the first line of ``ffmpeg -version`` or a readable fallback."""
    try:
        result = subprocess.run(
            [ffmpeg, "-hide_banner", "-version"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "未知"
    lines = result.stdout.splitlines()
    return lines[0].strip() if lines else "未知"


@dataclass(slots=True)
class CameraVideo:
    """Result of probing one recorded file."""

    camera_id: str
    path: Path
    frames: int
    width: int
    height: int
    avg_fps: float
    timestamps_ns: list[int]
    # Same frames on the monotonic clock. Lets the camera axis line up with
    # cap/timestamp_monotonic_ns without going through the wall clock (which
    # NTP can step mid-recording). None when the container carried no
    # monotonic stamps.
    timestamps_monotonic_ns: list[int] | None = None

    @property
    def duration_s(self) -> float:
        if len(self.timestamps_ns) < 2:
            return 0.0
        return (self.timestamps_ns[-1] - self.timestamps_ns[0]) / 1e9

    @property
    def actual_fps(self) -> float:
        span = self.duration_s
        return (self.frames - 1) / span if span > 0.0 else 0.0


def probe_frames(ffprobe: str, path: Path) -> list[int]:
    """Read every decoded frame pts and convert it to absolute Unix nanoseconds."""
    try:
        result = subprocess.run(
            [
                ffprobe,
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "frame=pts_time",
                "-of",
                "csv=p=0",
                str(path),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=300.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"ffprobe 读取 {path.name} 时间戳失败：{exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or "无输出"
        raise RuntimeError(f"ffprobe 读取 {path.name} 失败：{detail}")

    timestamps: list[int] = []
    for line in result.stdout.splitlines():
        text = line.strip().split(",")[0].strip()
        if not text or text.upper() == "N/A":
            continue
        try:
            seconds = float(text)
        except ValueError:
            continue
        timestamps.append(int(round(seconds * 1e9)))
    return timestamps


def probe_stream(ffprobe: str, path: Path) -> dict[str, object]:
    try:
        result = subprocess.run(
            [
                ffprobe,
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=width,height,avg_frame_rate,r_frame_rate",
                "-of",
                "json",
                str(path),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
        payload = json.loads(result.stdout or "{}")
    except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError):
        return {}
    streams = payload.get("streams") or []
    return streams[0] if streams else {}


def parse_rate(text: object) -> float:
    """Parse ffprobe's ``num/den`` frame-rate string."""
    if not isinstance(text, str) or "/" not in text:
        return 0.0
    numerator, _, denominator = text.partition("/")
    try:
        top = float(numerator)
        bottom = float(denominator)
    except ValueError:
        return 0.0
    return top / bottom if bottom else 0.0


def extract_jpeg_frames(buffer: bytearray) -> list[bytes]:
    """Pull complete JPEGs out of ``buffer``, consuming them in place.

    ``image2pipe`` is a bare concatenation with no framing of its own, so the
    only reliable delimiters are the SOI/EOI markers.  A partial frame at the
    tail is left in the buffer for the next read; anything before the first SOI
    (a stray marker from a torn write) is dropped.
    """
    frames: list[bytes] = []
    while True:
        start = buffer.find(b"\xff\xd8")
        if start < 0:
            buffer.clear()
            break
        end = buffer.find(b"\xff\xd9", start + 2)
        if end < 0:
            if start:
                del buffer[:start]
            break
        frames.append(bytes(buffer[start:end + 2]))
        del buffer[:end + 2]
    return frames


class FfmpegCamera:
    """Record one camera through ffmpeg over V4L2."""

    def __init__(
        self,
        camera_id: str,
        device: str,
        output_dir: Path,
        width: int,
        height: int,
        fps: float,
        codec: str,
        ffmpeg: str,
        preview_fps: float = 0.0,
        preview_width: int = PREVIEW_WIDTH,
        preview_quality: int = PREVIEW_QUALITY,
    ) -> None:
        self.camera_id = camera_id
        self.device = device
        self.output_dir = output_dir
        self.width = width
        self.height = height
        self.fps = fps
        self.codec = codec
        self.ffmpeg = ffmpeg
        self.ffprobe = find_ffprobe(ffmpeg)
        self.video_path = output_dir / f"{camera_id}.mkv"
        self.progress_path = output_dir / f"{camera_id}.progress"
        self.log_path = output_dir / f"{camera_id}.log"
        self.process: subprocess.Popen[bytes] | None = None
        self.failure: Exception | None = None
        self.start_unix_ns: int | None = None
        self.start_monotonic_ns: int | None = None
        self._log_handle = None
        # Live preview (display only; nothing here reaches the HDF5).
        self.preview_fps = max(0.0, float(preview_fps))
        self.preview_width = int(preview_width)
        self.preview_quality = int(preview_quality)
        self.preview_frames = 0
        self.preview_bytes = 0
        self._preview_stop = threading.Event()
        self._preview_thread: threading.Thread | None = None
        self._preview_lock = threading.Lock()
        self._preview_latest: tuple[int, bytes] | None = None

    def _codec_args(self) -> list[str]:
        if self.codec == "copy":
            return ["-c:v", "copy"]
        if self.codec == "mjpeg":
            return ["-c:v", "mjpeg", "-q:v", "3"]
        if self.codec == "h264":
            return [
                "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                "-pix_fmt", "yuv420p",
            ]
        if self.codec == "hevc":
            return [
                "-c:v", "libx265", "-preset", "veryfast", "-crf", "24",
                "-pix_fmt", "yuv420p",
            ]
        raise RuntimeError(f"未知视频编码：{self.codec}")

    def command(self) -> list[str]:
        command = [self.ffmpeg, "-hide_banner"]
        # No ``-nostdin``: ffmpeg 7 with -nostdin swallows the first SIGINT and
        # never finalises the container (measured on Ubuntu 22.04 + ffmpeg
        # 7.0.2 static + UVC; 4.4.2 was unaffected). ``start()`` already gives
        # the child /dev/null as stdin, which is what -nostdin was protecting.
        command += [
            "-loglevel",
            "warning",
            "-nostats",
            "-y",
            "-f",
            "v4l2",
            "-use_wallclock_as_timestamps",
            "1",
            "-thread_queue_size",
            "1024",
            "-input_format",
            "mjpeg",
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            f"{self.fps:g}",
            "-i",
            self.device,
            "-map",
            "0:v:0",
            "-fps_mode",
            "passthrough",
            "-copyts",
            "-muxdelay",
            "0",
            "-muxpreload",
            "0",
        ]
        progress = str(self.progress_path)
        output = str(self.video_path)
        command += self._codec_args()
        command += ["-progress", progress, output]
        if self.preview_fps > 0:
            # Second output on the same input: a decimated MJPEG stream on
            # stdout.  ``-an`` keeps audio away, and ``-f image2pipe`` is what
            # makes ffmpeg concatenate JPEGs instead of writing one file.
            command += [
                "-map", "0:v:0",
                "-an",
                "-vf", f"fps={self.preview_fps:g},scale={self.preview_width}:-2",
                "-c:v", "mjpeg",
                "-q:v", str(self.preview_quality),
                "-f", "image2pipe",
                "pipe:1",
            ]
        return command

    def open(self) -> None:
        """Validate inputs and clear stale artifacts before start()."""
        if self.codec not in VIDEO_CODECS:
            raise RuntimeError(f"未知视频编码：{self.codec}")
        if not Path(self.device).exists():
            raise RuntimeError(
                f"PXCap {self.camera_id} 摄像头节点不存在：{self.device}"
            )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        for path in (self.progress_path, self.log_path):
            try:
                path.unlink()
            except FileNotFoundError:
                pass

    def start(self) -> None:
        if self.process is not None:
            raise RuntimeError(f"PXCap {self.camera_id} 摄像头已启动")
        self._log_handle = self.log_path.open("wb")
        self.start_unix_ns = time.time_ns()
        self.start_monotonic_ns = time.monotonic_ns()
        try:
            self.process = subprocess.Popen(
                self.command(),
                stdin=subprocess.DEVNULL,
                stdout=(
                    subprocess.PIPE
                    if self.preview_fps > 0
                    else subprocess.DEVNULL
                ),
                stderr=self._log_handle,
                start_new_session=True,
            )
        except OSError as exc:
            self._close_log()
            raise RuntimeError(
                f"PXCap {self.camera_id} 摄像头启动失败：{exc}"
            ) from exc
        RUNNING_PROCESSES.append(self.process)
        if self.preview_fps > 0:
            self._preview_stop.clear()
            self._preview_thread = threading.Thread(
                target=self._preview_reader,
                name=f"preview-{self.camera_id}",
                daemon=True,
            )
            self._preview_thread.start()
        log(
            f"[INFO] PXCap {self.camera_id} 摄像头已启动："
            f"{self.width}x{self.height}@{self.fps:g} "
            f"codec={self.codec} device={self.device}"
        )

    def _preview_reader(self) -> None:
        """Split the stdout MJPEG stream into frames and keep only the newest.

        The pipe has to be drained continuously or ffmpeg blocks on write and
        the *recording* stalls, so this thread reads even when nobody is
        watching the preview; the last frame simply gets overwritten.
        """
        process = self.process
        if process is None or process.stdout is None:
            return
        stream = process.stdout
        buffer = bytearray()
        while not self._preview_stop.is_set():
            try:
                chunk = stream.read(65536)
            except (OSError, ValueError):
                break
            if not chunk:
                break
            buffer += chunk
            for frame in extract_jpeg_frames(buffer):
                stamp = time.time_ns()
                with self._preview_lock:
                    self._preview_latest = (stamp, frame)
                self.preview_frames += 1
                self.preview_bytes += len(frame)
            if len(buffer) > PREVIEW_MAX_BUFFER:
                buffer.clear()
        try:
            stream.close()
        except (OSError, ValueError):
            pass

    def latest_preview(self) -> tuple[int, bytes] | None:
        """Newest ``(unix_ns, jpeg_bytes)``, or None before the first frame."""
        with self._preview_lock:
            return self._preview_latest

    def progress_signal(self) -> int:
        """Return a monotonically growing frame-count estimate.

        ffmpeg omits ``frame=`` for some stream-copy paths, even though it
        still reports ``out_time_us`` and ``total_size``. Readiness checks need
        evidence that new frames landed, not a specific progress key.
        """
        try:
            text = self.progress_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return 0
        frames = 0
        out_time_us = 0
        total_size = 0
        for line in text.splitlines():
            key, _, value = line.partition("=")
            key = key.strip()
            if key not in ("frame", "out_time_us", "total_size"):
                continue
            try:
                number = int(value.strip())
            except ValueError:
                continue
            if key == "frame":
                frames = number
            elif key == "out_time_us":
                out_time_us = number
            else:
                total_size = number
        if frames > 0:
            return frames
        if out_time_us > 0:
            return max(1, int(round(out_time_us * self.fps / 1e6)))
        # A growing container is enough to prove the input produced a frame.
        return max(0, total_size // 65536)

    def poll_health(self) -> None:
        """Raise if ffmpeg exited while the recording is running."""
        if self.failure is not None:
            raise RuntimeError(
                f"PXCap {self.camera_id} 摄像头失败：{self.failure}"
            )
        if self.process is None:
            return
        code = self.process.poll()
        if code is None:
            return
        detail = self.log_tail()
        self.failure = RuntimeError(
            f"ffmpeg 退出码 {code}" + (f"，日志：{detail}" if detail else "")
        )
        raise RuntimeError(
            f"PXCap {self.camera_id} 摄像头失败：{self.failure}"
        )

    def log_tail(self, limit: int = 3) -> str:
        try:
            text = self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        return " | ".join(lines[-limit:])

    def stop(self) -> None:
        """Finalise the container with SIGINT."""
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except (ProcessLookupError, OSError, ValueError):
                pass
            try:
                process.wait(timeout=STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                warn(
                    f"PXCap {self.camera_id} 摄像头未在 "
                    f"{STOP_TIMEOUT_S:g} 秒内退出，强制结束"
                )
                process.kill()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    warn(f"PXCap {self.camera_id} 摄像头进程无法结束")
        # ffmpeg exits 255 on a handled SIGINT ("Exiting normally, received
        # signal 2"), so 255 after our own interrupt is a clean stop, not a
        # failure. A camera that died mid-recording is caught by poll_health().
        if process.returncode not in (0, 255, -signal.SIGINT, 130):
            detail = self.log_tail()
            message = f"ffmpeg 退出码 {process.returncode}" + (
                f"，日志：{detail}" if detail else ""
            )
            self.failure = self.failure or RuntimeError(message)
            warn(f"PXCap {self.camera_id} 摄像头异常结束：{message}")
        # Only now is it safe to stop the reader: ffmpeg must be allowed to
        # finish writing to stdout, otherwise it blocks on a full pipe and never
        # gets to finalise the mkv.
        self._preview_stop.set()
        if self._preview_thread is not None:
            self._preview_thread.join(timeout=2.0)
            self._preview_thread = None
        if process in RUNNING_PROCESSES:
            RUNNING_PROCESSES.remove(process)
        self.process = None
        self._close_log()

    def close(self) -> None:
        self.stop()

    def _close_log(self) -> None:
        if self._log_handle is not None:
            try:
                self._log_handle.close()
            except OSError:
                pass
            self._log_handle = None

    def probe(self) -> CameraVideo:
        """Recover per-frame absolute timestamps and stream metadata."""
        if not self.video_path.is_file():
            raise RuntimeError(
                f"PXCap {self.camera_id} 摄像头未生成视频文件：{self.video_path}"
            )
        raw_pts = probe_frames(self.ffprobe, self.video_path)
        if not raw_pts:
            detail = self.log_tail()
            raise RuntimeError(
                f"PXCap {self.camera_id} 摄像头视频没有可读帧"
                + (f"（日志：{detail}）" if detail else "")
            )
        if raw_pts[0] <= WALLCLOCK_MIN_NS:
            raise RuntimeError(
                f"PXCap {self.camera_id} 摄像头时间戳不是墙钟时间"
                f"（首帧 pts_time={raw_pts[0] / 1e9:.6f} 秒）。"
                "-use_wallclock_as_timestamps 未生效，请检查 ffmpeg 版本与命令"
            )
        timestamps = raw_pts
        timestamps_monotonic: list[int] | None = None
        stream = probe_stream(self.ffprobe, self.video_path)
        return CameraVideo(
            camera_id=self.camera_id,
            path=self.video_path,
            frames=len(timestamps),
            width=int(stream.get("width") or 0),
            height=int(stream.get("height") or 0),
            avg_fps=parse_rate(stream.get("avg_frame_rate"))
            or parse_rate(stream.get("r_frame_rate")),
            timestamps_ns=timestamps,
            timestamps_monotonic_ns=timestamps_monotonic,
        )

