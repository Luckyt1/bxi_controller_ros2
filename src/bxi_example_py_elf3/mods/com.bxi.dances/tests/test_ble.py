from __future__ import annotations

import importlib.util
import sys
import time
import types
from pathlib import Path


MOD_DIR = Path(__file__).resolve().parents[1]
BLE_SPEC = importlib.util.spec_from_file_location("_dances_ble_test_module", MOD_DIR / "ble.py")
assert BLE_SPEC is not None and BLE_SPEC.loader is not None
ble = importlib.util.module_from_spec(BLE_SPEC)
sys.modules[BLE_SPEC.name] = ble
BLE_SPEC.loader.exec_module(ble)


class FakeLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def warning(self, message: str) -> None:
        self.messages.append(("warning", message))


class FakeWriter:
    instances: list["FakeWriter"] = []

    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs
        self.started = 0
        self.stopped = 0
        self.writes = 0
        FakeWriter.instances.append(self)

    def start(self) -> None:
        self.started += 1

    def stop(self) -> None:
        self.stopped += 1

    def write(self) -> bool:
        self.writes += 1
        return True


def setup_function() -> None:
    FakeWriter.instances.clear()
    ble._SHARED_WRITERS.clear()


def test_no_trigger_frames_do_not_start_writer() -> None:
    trigger = ble.BleFrameTrigger("dance", trigger_frames=(), _writer_factory=FakeWriter)

    trigger.start(FakeLogger())
    trigger.write_for_timestep(1000)

    assert FakeWriter.instances == []


def test_crossed_trigger_frame_writes_once_per_playback() -> None:
    trigger = ble.BleFrameTrigger(
        "dance", trigger_frames=(10, 20), _writer_factory=FakeWriter
    )
    trigger.start(FakeLogger())

    trigger.write_for_timestep(9)
    trigger.write_for_timestep(12)
    trigger.write_for_timestep(20)
    trigger.write_for_timestep(21)
    trigger.write_for_timestep(20)

    writer = FakeWriter.instances[0]
    assert writer.writes == 2


def test_reset_allows_same_frame_to_trigger_again() -> None:
    trigger = ble.BleFrameTrigger(
        "dance", trigger_frames=(10,), _writer_factory=FakeWriter
    )
    trigger.start(FakeLogger())

    trigger.write_for_timestep(9)
    trigger.write_for_timestep(11)
    trigger.reset()
    trigger.write_for_timestep(9)
    trigger.write_for_timestep(11)

    assert FakeWriter.instances[0].writes == 2


def test_close_stops_writer_and_next_start_uses_new_writer() -> None:
    trigger = ble.BleFrameTrigger(
        "dance", trigger_frames=(10,), _writer_factory=FakeWriter
    )

    trigger.start(FakeLogger())
    first = FakeWriter.instances[0]
    trigger.close()
    trigger.start(FakeLogger())

    assert first.stopped == 1
    assert len(FakeWriter.instances) == 2
    assert FakeWriter.instances[1].started == 1


def test_matching_ble_targets_share_writer_until_last_close() -> None:
    first = ble.BleFrameTrigger(
        "dance_1", trigger_frames=(10,), _writer_factory=FakeWriter
    )
    second = ble.BleFrameTrigger(
        "dance_2", trigger_frames=(20,), _writer_factory=FakeWriter
    )

    first.start(FakeLogger())
    second.start(FakeLogger())
    first.close()
    second.write_for_timestep(20)
    second.close()

    writer = FakeWriter.instances[0]
    assert len(FakeWriter.instances) == 1
    assert writer.started == 2
    assert writer.writes == 1
    assert writer.stopped == 1


def test_gatt_write_queue_is_non_blocking_when_full() -> None:
    writer = ble._BleGattWriter(
        "mac",
        "characteristic",
        queue_size=1,
        logger=FakeLogger(),
    )
    writer._thread = types.SimpleNamespace(is_alive=lambda: True)

    started_at = time.monotonic()
    assert writer.write() is True
    assert writer.write() is False

    assert time.monotonic() - started_at < 0.1


def test_missing_bleak_logs_warning_without_raising(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "bleak", None)
    logger = FakeLogger()
    writer = ble._BleGattWriter(
        "mac",
        "characteristic",
        logger=logger,
        reconnect_delay=0.0,
    )

    writer.start()
    writer._thread.join(timeout=1.0)

    assert not writer._thread.is_alive()
    assert writer._stop_event.is_set()
    assert any("cannot import bleak" in message for _, message in logger.messages)


def test_gatt_writer_uses_fake_bleak_client(monkeypatch) -> None:
    writes: list[tuple[str, bytes, bool]] = []
    active_writer: list[ble._BleGattWriter] = []

    class FakeBleakClient:
        def __init__(self, address, timeout) -> None:
            self.address = address
            self.timeout = timeout
            self.is_connected = False

        async def connect(self) -> None:
            self.is_connected = True

        async def disconnect(self) -> None:
            self.is_connected = False

        async def write_gatt_char(self, characteristic, payload, *, response) -> None:
            writes.append((characteristic, payload, response))
            active_writer[0].stop()

    fake_bleak = types.ModuleType("bleak")
    fake_bleak.BleakClient = FakeBleakClient
    monkeypatch.setitem(sys.modules, "bleak", fake_bleak)
    writer = ble._BleGattWriter(
        "mac",
        "characteristic",
        b"\x01",
        logger=FakeLogger(),
        reconnect_delay=0.0,
    )
    active_writer.append(writer)

    assert writer.write() is True
    writer._thread.join(timeout=1.0)

    assert not writer._thread.is_alive()
    assert writes == [("characteristic", b"\x01", False)]
