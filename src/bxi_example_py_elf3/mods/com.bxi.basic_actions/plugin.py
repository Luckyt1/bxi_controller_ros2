from bxi_example_py_elf3.policies import (
    HumanoidGaitPolicyLiteIsaaclab,
    DanceMotionPolicyGravityIsaaclabV3,
    NormalMotionPolicyMjlab,
)
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .amp_run_state import AmpRunState
from .dance_state import DanceState
from .initial_pos_state import InitialPosState
from .imu_protection_state import ImuProtectionState
from .normal_run_state import NormalRunState
from .normal_state import NormalState
from .pd_brake_state import PdBrakeState
from .recover_state import RecoverState
from .zero_torque_state import ZeroTorqueState


NORMAL_POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab](
    "com.bxi.basic_actions/normal_policy"
)
AMP_RUN_POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab](
    "com.bxi.basic_actions/amp_run_policy"
)
NORMAL_RUN_POLICY = ResourceKey[NormalMotionPolicyMjlab](
    "com.bxi.basic_actions/normal_run_policy"
)
DANCE_POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3](
    "com.bxi.basic_actions/dance_policy"
)
RECOVER_POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3](
    "com.bxi.basic_actions/recover_policy"
)
RECOVER_FACE_POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3](
    "com.bxi.basic_actions/recover_face_policy"
)


def _load_normal_policy(
    context: ResourceLoadContext,
) -> HumanoidGaitPolicyLiteIsaaclab:
    return HumanoidGaitPolicyLiteIsaaclab(
        str(context.asset("assets/amp_terrain.onnx"))
    )


def _load_amp_run_policy(
    context: ResourceLoadContext,
) -> HumanoidGaitPolicyLiteIsaaclab:
    return HumanoidGaitPolicyLiteIsaaclab(
        str(context.asset("assets/amp_run.onnx"))
    )


def _load_normal_run_policy(context: ResourceLoadContext) -> NormalMotionPolicyMjlab:
    return NormalMotionPolicyMjlab(str(context.asset("assets/model_normal.onnx")))


def _load_dance_policy(
    context: ResourceLoadContext,
) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/shuishou.npz")),
        str(context.asset("assets/shuishou.onnx")),
        start_frame=60,
        fixed_pos=True,
    )


def _load_recover_policy(
    context: ResourceLoadContext,
) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/getup_back.npz")),
        str(context.asset("assets/getup_back.onnx")),
        start_frame=0,
    )


def _load_recover_face_policy(
    context: ResourceLoadContext,
) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/getup_face.npz")),
        str(context.asset("assets/getup_face.onnx")),
        start_frame=0,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(NORMAL_POLICY, _load_normal_policy)
    context.register_resource(AMP_RUN_POLICY, _load_amp_run_policy, policy="on_demand")
    context.register_resource(
        NORMAL_RUN_POLICY, _load_normal_run_policy, policy="on_demand"
    )
    context.register_resource(DANCE_POLICY, _load_dance_policy, policy="on_demand")
    context.register_resource(RECOVER_POLICY, _load_recover_policy, policy="on_demand")
    context.register_resource(
        RECOVER_FACE_POLICY,
        _load_recover_face_policy,
        policy="on_demand",
    )

    normal_policy = context.resource(NORMAL_POLICY)
    amp_run_policy = context.resource(AMP_RUN_POLICY)
    normal_run_policy = context.resource(NORMAL_RUN_POLICY)
    dance_policy = context.resource(DANCE_POLICY)
    recover_policy = context.resource(RECOVER_POLICY)
    recover_face_policy = context.resource(RECOVER_FACE_POLICY)

    return ModDefinition(
        state_factories={
            "normal": lambda state: NormalState(
                state.name, state.state_id, normal_policy
            ),
            "zero_torque": lambda state: ZeroTorqueState(
                state.name, state.state_id
            ),
            "imu_protection": lambda state: ImuProtectionState(
                state.name,
                state.state_id,
                kp=state.float_param("kp", 20.0),
                kd=state.float_param("kd", 1.0),
            ),
            "pd_brake": lambda state: PdBrakeState(
                state.name, state.state_id, normal_policy
            ),
            "initial_pos": lambda state: InitialPosState(
                state.name, state.state_id
            ),
            "dance": lambda state: DanceState(
                state.name,
                state.state_id,
                dance_policy,
                start_frame=state.int_param("start_frame", 100),
            ),
            "recover": lambda state: RecoverState(
                state.name,
                state.state_id,
                recover_policy,
                recover_face_policy,
            ),
            "amp_run": lambda state: AmpRunState(
                state.name, state.state_id, amp_run_policy
            ),
            "normal_run": lambda state: NormalRunState(
                state.name, state.state_id, normal_run_policy
            ),
        }
    )
