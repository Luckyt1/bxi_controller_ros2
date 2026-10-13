from __future__ import annotations

import asyncio
import queue
import threading
import time
from typing import Any, Callable, Protocol


DEFAULT_MAC = "E0:72:A1:D8:39:8E"
DEFAULT_CHARACTERISTIC = "0000ee01-0000-1000-8000-00805f9b34fb"
DEFAULT_PAYLOAD = b"\x01"


_SharedWriterKey = tuple[str, str, bytes, bool, int]
_SHARED_WRITERS: dict[_SharedWriterKey, tuple["_BleGattWriter", int]] = {}
_SHARED_WRITERS_LOCK = threading.Lock()


class _Logger(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class _BleGattWriter:
    def __init__(
        self,
        address: str,
        characteristic: str,
        payload: bytes = DEFAULT_PAYLOAD,
        *,
        write_with_response: bool = False,
        logger: _Logger | Callable[[str, str], None] | None = None,
        connect_timeout: float = 10.0,
        reconnect_delay: float = 1.0,
        queue_size: int = 32,
    ) -> None:
        self.address = address
        self.characteristic = characteristic
        self.payload = payload or DEFAULT_PAYLOAD
        self.write_with_response = write_with_response
        self.connect_timeout = connect_timeout
        self.reconnect_delay = reconnect_delay
        self._logger = logger
        self._queue: queue.Queue[bytes | None] = queue.Queue(maxsize=queue_size)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_log_at: dict[str, float] = {}

    def start(self) -> None:
        if self._stop_event.is_set():
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._thread_main,
            name=f"ble-gatt-writer-{self.address}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass

    def write(self) -> bool:
        if self._stop_event.is_set():
            return False
        if self._thread is None or not self._thread.is_alive():
            self.start()
        try:
            self._queue.put_nowait(self.payload)
            return True
        except queue.Full:
            self._log_throttled(
                "queue_full",
                "warning",
                "BLE write queue is full, dropping payload",
            )
            return False

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._run())
        except Exception as exc:
            self._log("warning", f"BLE writer stopped: {exc}")

    async def _run(self) -> None:
        try:
            from bleak import BleakClient
        except Exception as exc:
            self._log("warning", f"BLE disabled, cannot import bleak: {exc}")
            self._stop_event.set()
            return

        client = BleakClient(self.address, timeout=self.connect_timeout)
        pending: bytes | None = None

        try:
            while not self._stop_event.is_set():
                if not getattr(client, "is_connected", False):
                    if not await self._ensure_connected(client):
                        await asyncio.sleep(self.reconnect_delay)
                        continue

                if pending is None:
                    try:
                        pending = self._queue.get(True, 0.1)
                    except queue.Empty:
                        continue

                if pending is None or self._stop_event.is_set():
                    break

                try:
                    await client.write_gatt_char(
                        self.characteristic,
                        pending,
                        response=self.write_with_response,
                    )
                    pending = None
                except Exception as exc:
                    self._log_throttled(
                        "write_failed",
                        "warning",
                        f"BLE write failed: {exc}",
                    )
                    await self._disconnect(client)
                    await asyncio.sleep(self.reconnect_delay)
        finally:
            await self._disconnect(client)

    async def _ensure_connected(self, client: Any) -> bool:
        if getattr(client, "is_connected", False):
            return True
        try:
            await client.connect()
            self._log("info", f"BLE connected: {self.address}")
            return True
        except Exception as exc:
            self._log_throttled(
                "connect_failed",
                "warning",
                f"BLE connect failed for {self.address}: {exc}",
            )
            return False

    async def _disconnect(self, client: Any) -> None:
        if not getattr(client, "is_connected", False):
            return
        try:
            await client.disconnect()
            self._log("info", f"BLE disconnected: {self.address}")
        except Exception as exc:
            self._log_throttled(
                "disconnect_failed",
                "warning",
                f"BLE disconnect failed for {self.address}: {exc}",
            )

    def _log_throttled(
        self, key: str, level: str, message: str, interval: float = 5.0
    ) -> None:
        now = time.monotonic()
        if now - self._last_log_at.get(key, 0.0) < interval:
            return
        self._last_log_at[key] = now
        self._log(level, message)

    def _log(self, level: str, message: str) -> None:
        logger = self._logger
        if logger is None:
            return
        try:
            if callable(logger):
                logger(level, message)
                return
            log_method = getattr(logger, level, None)
            if callable(log_method):
                log_method(message)
        except Exception:
            return


class BleFrameTrigger:
    def __init__(
        self,
        state_name: str,
        *,
        trigger_frames: tuple[int, ...],
        mac: str = DEFAULT_MAC,
        characteristic: str = DEFAULT_CHARACTERISTIC,
        _writer_factory: type[_BleGattWriter] | Callable[..., _BleGattWriter] = _BleGattWriter,
    ) -> None:
        self.state_name = state_name
        self.trigger_frames = tuple(sorted({int(frame) for frame in trigger_frames}))
        self.mac = str(mac).strip()
        self.characteristic = str(characteristic).strip()
        self._writer_factory = _writer_factory
        self._writer: _BleGattWriter | None = None
        self._writer_key: _SharedWriterKey | None = None
        self._triggered_frames: set[int] = set()
        self._last_frame: int | None = None

    def start(self, logger: _Logger | Callable[[str, str], None] | None) -> None:
        if not self.trigger_frames or not self.mac or not self.characteristic:
            return
        if self._writer is None:
            self._writer_key = self._make_writer_key()
            self._writer = self._acquire_writer(self._writer_key, logger)
        self._writer.start()

    def reset(self) -> None:
        self._triggered_frames.clear()
        self._last_frame = None

    def write_for_timestep(self, timestep: float) -> None:
        if not self.trigger_frames or self._writer is None:
            return

        frame = int(timestep)
        if self._last_frame is None or frame <= self._last_frame:
            candidates = (frame,)
        else:
            candidates = range(self._last_frame + 1, frame + 1)

        for candidate in candidates:
            if candidate not in self.trigger_frames:
                continue
            if candidate in self._triggered_frames:
                continue
            self._writer.write()
            self._triggered_frames.add(candidate)

        self._last_frame = frame

    def close(self) -> None:
        if self._writer is None or self._writer_key is None:
            return
        self._release_writer(self._writer_key, self._writer)
        self._writer = None
        self._writer_key = None

    def _make_logger(
        self, logger: _Logger | Callable[[str, str], None] | None
    ) -> Callable[[str, str], None] | None:
        if logger is None:
            return None

        def log(level: str, message: str) -> None:
            prefixed = f"[{self.state_name}] {message}"
            if callable(logger):
                logger(level, prefixed)
                return
            log_method = getattr(logger, level, None)
            if callable(log_method):
                log_method(prefixed)

        return log

    def _make_writer_key(self) -> _SharedWriterKey:
        return (
            self.mac,
            self.characteristic,
            DEFAULT_PAYLOAD,
            False,
            id(self._writer_factory),
        )

    def _acquire_writer(
        self,
        key: _SharedWriterKey,
        logger: _Logger | Callable[[str, str], None] | None,
    ) -> _BleGattWriter:
        with _SHARED_WRITERS_LOCK:
            existing = _SHARED_WRITERS.get(key)
            if existing is not None:
                writer, ref_count = existing
                _SHARED_WRITERS[key] = (writer, ref_count + 1)
                return writer

            writer = self._writer_factory(
                self.mac,
                self.characteristic,
                DEFAULT_PAYLOAD,
                write_with_response=False,
                logger=self._make_logger(logger),
            )
            _SHARED_WRITERS[key] = (writer, 1)
            return writer

    @staticmethod
    def _release_writer(key: _SharedWriterKey, writer: _BleGattWriter) -> None:
        should_stop = False
        with _SHARED_WRITERS_LOCK:
            existing = _SHARED_WRITERS.get(key)
            if existing is None:
                should_stop = True
            else:
                shared_writer, ref_count = existing
                if shared_writer is not writer:
                    return
                if ref_count <= 1:
                    del _SHARED_WRITERS[key]
                    should_stop = True
                else:
                    _SHARED_WRITERS[key] = (shared_writer, ref_count - 1)

        if should_stop:
            writer.stop()
