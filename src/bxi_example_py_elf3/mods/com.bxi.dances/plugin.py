from functools import partial

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
    ResourceHandle,
    StateBuildContext,
)
from bxi_example_py_elf3.policies import (
    DanceMotionPolicyGravityIsaaclabV2,
    DanceMotionPolicyGravityIsaaclabV3,
)

from .ble import BleFrameTrigger
from .state import DancePolicy, DanceState


MOTIONS = {
    "dance_part_3": ("face3.npz", "face3_17k.onnx", DanceMotionPolicyGravityIsaaclabV3),
    "dance_part_5": ("face4.npz", "face4_4k.onnx", DanceMotionPolicyGravityIsaaclabV3),
    "dance_part_6": ("jixiewu.npz", "jixiewu2.onnx", DanceMotionPolicyGravityIsaaclabV2),
    "dance_part_7": ("lichenxi.npz", "lichenxi3.onnx", DanceMotionPolicyGravityIsaaclabV2),
}


def _build_state(
    state: StateBuildContext,
    *,
    policy: ResourceHandle[DancePolicy],
) -> DanceState:
    trigger_frames = state.param("ble_trigger_frames", list, [])
    if any(type(frame) is not int or frame < 0 for frame in trigger_frames):
        raise ValueError(
            f"state '{state.name}' param 'ble_trigger_frames' must contain non-negative integers"
        )
    trigger = BleFrameTrigger(
        state.name,
        trigger_frames=tuple(trigger_frames) if state.bool_param("ble_enabled", True) else (),
        mac=state.string_param("ble_mac", "E0:72:A1:D8:39:8E"),
        characteristic=state.string_param(
            "ble_characteristic", "0000ee01-0000-1000-8000-00805f9b34fb"
        ),
    )
    return DanceState(
        state.name,
        state.state_id,
        policy,
        ble_trigger=trigger,
        start_frame=state.int_param("start_frame", 10),
        end_trim_frames=state.int_param("end_trim_frames", 0),
    )


def _load_policy(
    context: ResourceLoadContext,
    *,
    motion: str,
    model: str,
    policy_type: type[DancePolicy],
) -> DancePolicy:
    return policy_type(
        str(context.asset(f"assets/{motion}")),
        str(context.asset(f"assets/{model}")),
        start_frame=10,
        fixed_pos=True,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    factories = {}
    for name, (motion, model, policy_type) in MOTIONS.items():
        key = ResourceKey[DancePolicy](f"{context.mod_id}/{name}/policy")
        context.register_resource(
            key,
            partial(_load_policy, motion=motion, model=model, policy_type=policy_type),
            policy="on_demand",
        )
        handle = context.resource(key)
        factories[name] = partial(_build_state, policy=handle)
    return ModDefinition(state_factories=factories)
