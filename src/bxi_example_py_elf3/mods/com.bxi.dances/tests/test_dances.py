"""Run with the package source on PYTHONPATH and the ROS environment sourced."""

import importlib
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.framework import StateMachineInspector
from bxi_example_py_elf3.framework.inference import InferenceFrame
from bxi_example_py_elf3.framework.joints import JointStateBuffer
from bxi_example_py_elf3.framework.mod_api import ModLoadContext, StateBuildContext
from bxi_example_py_elf3.framework.runtime.resource_manager import ResourceManager
from bxi_example_py_elf3.policies import (
    DanceMotionPolicyGravityIsaaclabV2,
    DanceMotionPolicyGravityIsaaclabV3,
)


MOD_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = MOD_ROOT.parents[1]
MOD_ID = "com.bxi.dances"
BASE_ID = "com.bxi.basic_actions"
package = ModuleType("_dances_under_test")
package.__path__ = [str(MOD_ROOT)]
sys.modules[package.__name__] = package
plugin = importlib.import_module(f"{package.__name__}.plugin")


@pytest.fixture
def definitions():
    resources = ResourceManager()
    try:
        definition = plugin.create_mod(ModLoadContext(MOD_ID, MOD_ROOT, resources))
        yield definition, resources
    finally:
        resources.close()


@pytest.mark.parametrize(
    "name,expected,start_frame",
    [("dance_part_3", (900,), 10), ("dance_part_5", (1000,), 50),
     ("dance_part_6", (), 10), ("dance_part_7", (), 10)],
)
def test_manifest_ble_trigger_defaults(definitions, name, expected, start_frame):
    definition, _ = definitions
    manifest = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    context = StateBuildContext(name, 1, manifest["states"][name]["params"])
    state = definition.state_factories[name](context)
    context.finish()
    assert state._ble_trigger.trigger_frames == expected
    assert state.start_frame == start_frame


@pytest.mark.parametrize(
    "params,expected",
    [({}, ()), ({"ble_trigger_frames": []}, ()),
     ({"ble_trigger_frames": [850, 950, 850]}, (850, 950)),
     ({"ble_trigger_frames": [850], "ble_enabled": False}, ())],
)
def test_configured_ble_frames_control_sends(definitions, params, expected):
    definition, _ = definitions
    context = StateBuildContext("dance_part_3", 1, params)
    state = definition.state_factories["dance_part_3"](context)
    context.finish()
    trigger = state._ble_trigger
    assert trigger.trigger_frames == expected
    writer = trigger._writer = Mock()
    for frame in (10, 849, 850, 850, 900, 949, 950, 1000):
        trigger.write_for_timestep(frame)
        assert writer.write.call_count == sum(target <= frame for target in expected)


@pytest.mark.parametrize("frames", [900, "900", None, [-1], [1.5], [True], ["900"]])
def test_invalid_ble_frames_are_rejected(definitions, frames):
    definition, _ = definitions
    context = StateBuildContext("dance_part_3", 1, {"ble_trigger_frames": frames})
    with pytest.raises(ValueError, match="ble_trigger_frames"):
        definition.state_factories["dance_part_3"](context)


@pytest.mark.parametrize("start_frame", [-1, True, "10", 1.5, None])
def test_invalid_start_frame_is_rejected(definitions, start_frame):
    definition, _ = definitions
    context = StateBuildContext("dance_part_3", 1, {"start_frame": start_frame})
    with pytest.raises(ValueError, match="start_frame|negative|integer"):
        definition.state_factories["dance_part_3"](context)


def build_state(definitions, name, **overrides):
    definition, _ = definitions
    manifest = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    params = dict(manifest["states"][name].get("params", {}))
    params["ble_enabled"] = False
    params.update(overrides)
    context = StateBuildContext(f"{MOD_ID}/{name}", 1, params)
    state = definition.state_factories[name](context)
    context.finish()
    state._bind_logger(Mock())
    handle = state.required_resources[0]
    if handle.status == "unloaded":
        handle.request()
        deadline = time.monotonic() + 10.0
        while handle.status == "loading" and time.monotonic() < deadline:
            time.sleep(0.01)
    assert handle.status == "ready", handle.error
    return state


def control_context(policy):
    joints = JointStateBuffer(policy.joint_contract.observation)
    joints.position[:] = policy.motioninputpos[policy._start_frame]
    frame = InferenceFrame(joints.view, np.array([1.0, 0, 0, 0]), np.zeros(3))

    def preheat(model):
        model.reset(frame)
        model.step(frame, 0.02, advance=False)

    return SimpleNamespace(
        inference_frame=frame,
        current_quat_xyzw=np.array([0, 0, 0, 1.0]),
        preheat_model=preheat,
        is_orientation_unsafe=Mock(return_value=False),
        request_state=Mock(),
        set_motor_target=Mock(),
    )


@pytest.mark.parametrize(
    "name,expected_policy",
    [
        ("dance_part_3", DanceMotionPolicyGravityIsaaclabV3),
        ("dance_part_5", DanceMotionPolicyGravityIsaaclabV3),
        ("dance_part_6", DanceMotionPolicyGravityIsaaclabV2),
        ("dance_part_7", DanceMotionPolicyGravityIsaaclabV2),
    ],
)
def test_real_models_play_pause_finish_and_reenter(definitions, name, expected_policy):
    state = build_state(definitions, name)
    policy = state.policy
    assert type(policy) is expected_policy
    ctx = control_context(policy)
    state.on_prepare(ctx, Mock())
    assert policy.fixed_pos
    assert state._motion_frame == state.start_frame
    assert policy.num_actions == 29

    # Entry blending advances the destination just like the original project.
    state.sample_running_frame(ctx, 0.02, advance=True)
    assert state._motion_frame == state.start_frame + 1
    state.on_enter(ctx)
    assert state._motion_frame == state.start_frame + 1  # no rewind after the blend
    state.on_update(ctx, 0.02)
    assert ctx.set_motor_target.called
    assert np.isfinite(policy.output.joints.position).all()

    assert state.on_action(ctx, "toggle_pause")
    before = state._motion_frame
    state.on_update(ctx, 0.02)
    state.sample_running_frame(ctx, 0.02, advance=True)
    assert state._motion_frame == before
    assert not state.on_action(ctx, "unknown")
    assert state.on_action(ctx, "toggle_pause")

    # Exercise every retained reference frame and stop exactly at the trim boundary.
    while not policy.finished(trim=state.end_trim_frames):
        state.sample_running_frame(ctx, 0.02, advance=True)
        assert np.isfinite(policy.output.joints.position).all()
    assert state._motion_frame == len(policy.motioninputpos) - state.end_trim_frames
    assert state.sample_running_frame(ctx, 0.02, advance=True) is None
    state.on_update(ctx, 0.02)
    assert ctx.request_state.call_args.args == (f"{BASE_ID}/normal",)
    assert ctx.request_state.call_args.kwargs["transition"]["sample_from"] is False
    assert ctx.request_state.call_args.kwargs["transition"]["duration"] == 0.3

    state.playing = False
    state.on_prepare(ctx, Mock())
    assert state.playing
    assert state._motion_frame == state.start_frame
    assert not policy.finished()
    state.on_unbind(ctx)


@pytest.mark.parametrize("name,last_frame", [("dance_part_3", 1325), ("dance_part_5", 1299)])
def test_trim_keeps_ble_trigger_and_original_model_reference_window(definitions, name, last_frame):
    state = build_state(definitions, name, end_trim_frames=100)
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    manifest = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    trigger_frame = manifest["states"][name]["params"]["ble_trigger_frames"][0]
    assert trigger_frame < last_frame
    assert state.end_trim_frames == 100
    # The original model still sees the uncut reference, including its lookahead.
    assert state.policy._end_frame == len(state.policy.motioninputpos) - 1
    state.policy.advance((last_frame - state.start_frame) / 50.0)
    assert state.sample_running_frame(ctx, 0.02, advance=True) is not None
    assert state.sample_running_frame(ctx, 0.02, advance=True) is None
    state.on_update(ctx, 0.02)
    ctx.request_state.assert_called_once()


@pytest.mark.parametrize("name", plugin.MOTIONS)
@pytest.mark.parametrize("trim", [0, 1, 50])
def test_each_dance_accepts_custom_tail_trim(definitions, name, trim):
    state = build_state(definitions, name, end_trim_frames=trim)
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    last_frame = len(state.policy.motioninputpos) - trim - 1
    # Advance with the real 50 Hz tick, avoiding a fractional large time jump.
    for _ in range(last_frame - state.start_frame):
        state.policy.advance(0.02)
    assert state.sample_running_frame(ctx, 0.02, advance=True) is not None
    assert state.sample_running_frame(ctx, 0.02, advance=True) is None
    state.on_update(ctx, 0.02)
    assert ctx.request_state.call_args.args == (f"{BASE_ID}/normal",)
    state.on_unbind(ctx)


@pytest.mark.parametrize("name", plugin.MOTIONS)
@pytest.mark.parametrize("start_frame", [0, 60])
def test_each_dance_accepts_custom_start_frame_with_tail_trim(
    definitions, name, start_frame
):
    state = build_state(definitions, name, start_frame=start_frame, end_trim_frames=50)
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    assert state.start_frame == start_frame
    assert state._motion_frame == start_frame
    assert state.policy._start_frame == start_frame
    assert state.policy._frame == start_frame
    assert state.policy._end_frame == len(state.policy.motioninputpos) - 1

    last_frame = len(state.policy.motioninputpos) - state.end_trim_frames - 1
    state.policy.advance((last_frame - start_frame) / 50.0)
    state._motion_frame = float(last_frame)
    assert state.sample_running_frame(ctx, 0.02, advance=True) is not None
    assert state.sample_running_frame(ctx, 0.02, advance=True) is None
    state.on_update(ctx, 0.02)
    assert ctx.request_state.call_args.args == (f"{BASE_ID}/normal",)
    state.on_unbind(ctx)


def test_custom_start_frame_pause_and_reenter_reset_playback(definitions):
    state = build_state(definitions, "dance_part_3", start_frame=60, end_trim_frames=50)
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    state.sample_running_frame(ctx, 0.02, advance=True)
    assert state._motion_frame == 61

    assert state.on_action(ctx, "toggle_pause")
    before = state._motion_frame
    state.sample_running_frame(ctx, 0.02, advance=True)
    assert state._motion_frame == before

    state.on_prepare(ctx, Mock())
    assert state.playing
    assert state._motion_frame == 60
    assert state.policy._frame == 60
    state.on_unbind(ctx)


def test_start_frame_and_trim_must_leave_a_playable_frame(definitions):
    state = build_state(definitions, "dance_part_3", start_frame=60, end_trim_frames=0)
    ctx = control_context(state.policy)
    one_frame_trim = len(state.policy.motioninputpos) - state.start_frame - 1

    state.end_trim_frames = one_frame_trim
    state.on_prepare(ctx, Mock())
    assert state.sample_running_frame(ctx, 0.02, advance=True) is not None
    assert state.sample_running_frame(ctx, 0.02, advance=True) is None
    state.on_unbind(ctx)

    state = build_state(definitions, "dance_part_3", start_frame=60, end_trim_frames=0)
    ctx = control_context(state.policy)
    state.end_trim_frames = len(state.policy.motioninputpos) - state.start_frame
    with pytest.raises(ValueError, match="playable frame"):
        state.on_prepare(ctx, Mock())


def test_start_trimmed_ble_frame_is_not_replayed_from_zero(definitions):
    state = build_state(
        definitions,
        "dance_part_3",
        start_frame=60,
        end_trim_frames=50,
        ble_enabled=True,
        ble_trigger_frames=[20, 65],
    )
    writer = Mock()
    state._ble_trigger._writer = writer
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    state._ble_trigger._writer = writer

    state.on_update(ctx, 0.02)
    writer.write.assert_not_called()
    assert state._motion_frame == 61

    for _ in range(5):
        state.on_update(ctx, 0.02)

    writer.write.assert_called_once()
    assert 20 not in state._ble_trigger._triggered_frames
    assert 65 in state._ble_trigger._triggered_frames
    state.on_unbind(ctx)


def test_trim_can_be_disabled_and_rejects_invalid_ranges(definitions):
    state = build_state(definitions, "dance_part_3", end_trim_frames=0)
    assert state.end_trim_frames == 0
    ctx = control_context(state.policy)
    state.end_trim_frames = len(state.policy.motioninputpos) - 10
    with pytest.raises(ValueError, match="playable frame"):
        state.on_prepare(ctx, Mock())
    with pytest.raises(ValueError, match="negative"):
        build_state(definitions, "dance_part_5", end_trim_frames=-1)


def test_unsafe_orientation_preempts_motor_output(definitions):
    state = build_state(definitions, "dance_part_3")
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    ctx.is_orientation_unsafe.return_value = True
    state.on_update(ctx, 0.02)
    ctx.request_state.assert_called_once_with(f"{BASE_ID}/zero_torque", trigger="safety")
    ctx.set_motor_target.assert_not_called()


def test_ble_uses_motion_time_and_skips_pause_and_blends(definitions):
    state = build_state(definitions, "dance_part_3")
    trigger = state._ble_trigger = Mock()
    ctx = control_context(state.policy)
    state.on_prepare(ctx, Mock())
    trigger.start.assert_called_once()
    state.sample_running_frame(ctx, 0.02, advance=True)
    trigger.write_for_timestep.assert_not_called()
    state.on_enter(ctx)
    state.on_update(ctx, 0.02)
    trigger.write_for_timestep.assert_called_once_with(11.0)
    state.on_action(ctx, "toggle_pause")
    state.on_update(ctx, 0.02)
    assert trigger.write_for_timestep.call_count == 1
    state.on_unbind(ctx)
    trigger.close.assert_called_once()


def test_all_installed_mods_compose_without_event_collisions():
    inspector = StateMachineInspector(
        PACKAGE_ROOT / "config/elf3_state_machine.yaml",
        built_in_mod_root=PACKAGE_ROOT / "mods",
        extra_mod_roots=[],
    )
    config = inspector.config
    assert config["transition_profiles"][f"{MOD_ID}/dance_entry"]["duration"] == 0.3
    states = config["states"]
    normal = states[f"{BASE_ID}/normal"]["transitions"]["on_event"]
    for number, value in [(3, 55), (5, 56), (6, 57), (7, 58)]:
        event = f"{MOD_ID}/dance_{number}"
        name = f"{MOD_ID}/dance_part_{number}"
        assert config["remote_events"][event] == {"slot": "btn_10", "value": value}
        assert normal[event]["to"] == name
        routes = states[name]["transitions"]["on_event"]
        assert routes[f"{BASE_ID}/normal"]["to"] == f"{BASE_ID}/normal"
        assert routes[f"{BASE_ID}/normal"]["transition"]["duration"] == 0.3
        assert routes[f"{BASE_ID}/zero_torque"]["transition"] == "instant"
        assert routes[f"{BASE_ID}/toggle_pause"]["action"] == "toggle_pause"
        owners = [
            key for key, binding in config["remote_events"].items()
            if binding == {"slot": "btn_10", "value": value}
        ]
        assert owners == [event]


def test_remote_config_routes_each_shortcut_to_its_mod_event():
    remote = yaml.safe_load(
        (PACKAGE_ROOT.parent / "remote_controller/config/xbox_default.yaml").read_text()
    )
    manifest = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    signals = remote["sources"]["keyboard"]["signals"]
    for number, button, key in [(3, "y", "4"), (5, "b", "5"), (6, "a", "8"), (7, "x", "9")]:
        event = manifest["events"][f"dance_{number}"]
        output = f"{event['slot']}={event['value']}"
        rules = [rule for rule in remote["outputs"]["level"] if rule["output"] == output]
        assert len(rules) == 1
        gamepad, keyboard = rules[0]["when"]["any"]
        assert "trigger.right_event" in gamepad
        assert f"button.{button}_event" in gamepad
        assert keyboard == [f"keyboard.dance_{number}_event"]
        assert signals[f"keyboard.dance_{number}"]["key"] == key
