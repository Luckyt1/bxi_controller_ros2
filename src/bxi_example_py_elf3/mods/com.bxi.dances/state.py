from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ResourceHandle,
    RobotControlContext,
    RobotControlState,
    StateBehavior,
)
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)
from bxi_example_py_elf3.policies import (
    DanceMotionPolicyGravityIsaaclabV2,
    DanceMotionPolicyGravityIsaaclabV3,
)

from .ble import BleFrameTrigger


DancePolicy = DanceMotionPolicyGravityIsaaclabV2 | DanceMotionPolicyGravityIsaaclabV3


class DanceState(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    """Play a V2 or V3 motion using the target Mod lifecycle."""

    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[DancePolicy],
        *,
        ble_trigger: BleFrameTrigger,
        start_frame: int = 10,
        end_trim_frames: int = 0,
    ) -> None:
        if start_frame < 0:
            raise ValueError("start_frame must not be negative")
        if end_trim_frames < 0:
            raise ValueError("end_trim_frames must not be negative")
        super().__init__(name, state_id, resources=(policy,))
        self.start_frame = start_frame
        self.end_trim_frames = end_trim_frames
        self._policy = policy
        self.playing = True
        self._ble_trigger = ble_trigger
        self._motion_frame = float(self.start_frame)

    @property
    def policy(self) -> DancePolicy:
        return self._policy.get()

    def on_prepare(
        self, ctx: RobotControlContext, from_state: StateBehavior[RobotControlContext]
    ) -> None:
        if self.start_frame + self.end_trim_frames >= len(self.policy.motioninputpos):
            raise ValueError(
                "start_frame and end_trim_frames must leave at least one playable frame"
            )
        # Reset even when re-entering a previously paused dance. Blends sample
        # the destination before on_enter, so playback must resume here.
        self.playing = True
        self._motion_frame = float(self.start_frame)
        self.policy.configure_range(start_frame=self.start_frame)
        ctx.preheat_model(self.policy)
        self._ble_trigger.reset()
        self._ble_trigger.start(self.logger)

    def on_enter(self, ctx: RobotControlContext) -> None:
        self.playing = True

    def on_unbind(self, ctx: RobotControlContext) -> None:
        self._ble_trigger.close()

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._motor_frame_from_target(ctx, self.policy.output.joints)

    def sample_running_frame(
        self, ctx: RobotControlContext, dt: float, *, advance: bool
    ) -> MotorFrame | None:
        # Keep the original policy range so its command-window observations
        # remain unchanged; only stop playback before the unwanted tail.
        if self.policy.finished(trim=self.end_trim_frames):
            return None
        output = self.policy.step(
            ctx.inference_frame, dt, advance=self.playing and advance
        )
        if self.playing and advance:
            self._motion_frame += 50.0 * dt
        return self._motor_frame_from_target(ctx, output.joints)

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            ctx.request_state("com.bxi.basic_actions/zero_torque", trigger="safety")
            return
        if self.policy.finished(trim=self.end_trim_frames):
            ctx.request_state(
                "com.bxi.basic_actions/normal",
                trigger="motion_finished",
                transition={
                    "profile": "dual_running_blend",
                    "duration": 0.3,
                    "sample_from": False,
                },
            )
            return
        if self.playing:
            self._ble_trigger.write_for_timestep(self._motion_frame)
        self._apply_frame(ctx, self.sample_running_frame(ctx, dt, advance=True))

    def on_action(self, ctx: RobotControlContext, action_name: str) -> bool:
        if action_name != "toggle_pause":
            return False
        self.playing = not self.playing
        return True
