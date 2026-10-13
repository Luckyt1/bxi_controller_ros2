"""Regression coverage for the basic actions retained after gesture removal."""

import importlib
from pathlib import Path
import sys
from types import ModuleType

import yaml

from bxi_example_py_elf3.framework import StateMachineInspector
from bxi_example_py_elf3.framework.mod_api import ModLoadContext, StateBuildContext
from bxi_example_py_elf3.framework.runtime.resource_manager import ResourceManager


MOD_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = MOD_ROOT.parents[1]
MOD_ID = "com.bxi.basic_actions"
RETAINED_RESOURCES = {
    "normal": {"normal_policy"},
    "zero_torque": set(),
    "imu_protection": set(),
    "pd_brake": {"normal_policy"},
    "initial_pos": set(),
    "dance": {"dance_policy"},
    "recover": {"recover_policy", "recover_face_policy"},
    "amp_run": {"amp_run_policy"},
    "normal_run": {"normal_run_policy"},
}


def test_retained_factories_keep_resources_and_parameters():
    package = ModuleType("_basic_actions_under_test")
    package.__path__ = [str(MOD_ROOT)]
    sys.modules[package.__name__] = package
    plugin = importlib.import_module(f"{package.__name__}.plugin")
    manifest = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    resources = ResourceManager()
    try:
        definition = plugin.create_mod(ModLoadContext(MOD_ID, MOD_ROOT, resources))
        assert set(definition.state_factories) == set(RETAINED_RESOURCES)
        assert set(manifest["states"]) == set(RETAINED_RESOURCES)
        for index, (name, expected) in enumerate(RETAINED_RESOURCES.items()):
            context = StateBuildContext(
                f"{MOD_ID}/{name}", index, manifest["states"][name].get("params", {})
            )
            state = definition.state_factories[name](context)
            context.finish()
            assert {handle.key.id for handle in state.required_resources} == {
                f"{MOD_ID}/{resource}" for resource in expected
            }
            if name == "dance":
                assert state.start_frame == 100
    finally:
        resources.close()


def test_retained_input_and_safety_routes_compose_with_four_dances():
    inspector = StateMachineInspector(
        PACKAGE_ROOT / "config/elf3_state_machine.yaml",
        built_in_mod_root=PACKAGE_ROOT / "mods",
        extra_mod_roots=[],
    )
    config = inspector.config
    assert config["initial_state"] == f"{MOD_ID}/zero_torque"
    events = [
        ("normal", "btn_1", 1), ("navigation_control", "btn_1", 2),
        ("zero_torque", "btn_2", 1), ("pd_brake", "btn_3", 1),
        ("initial_pos", "btn_4", 1), ("dance", "btn_5", 1),
        ("recover", "btn_6", 1), ("normal_run", "btn_7", 1),
        ("amp_run", "btn_8", 1), ("toggle_pause", "btn_9", 1),
    ]
    for event, slot, value in events:
        assert config["remote_events"][f"{MOD_ID}/{event}"] == {
            "slot": slot, "value": value,
        }
    states = config["states"]
    for name in ("lie_down", "applause", "hello"):
        assert f"{MOD_ID}/{name}" not in states
        assert f"{MOD_ID}/{name}" not in config["remote_events"]
    for source in ("normal", "pd_brake", "initial_pos", "dance", "recover"):
        routes = states[f"{MOD_ID}/{source}"]["transitions"]["on_event"]
        assert routes[f"{MOD_ID}/zero_torque"]["to"] == f"{MOD_ID}/zero_torque"
    normal_routes = states[f"{MOD_ID}/normal"]["transitions"]["on_event"]
    for number in (3, 5, 6, 7):
        event = f"com.bxi.dances/dance_{number}"
        assert normal_routes[event]["to"] == f"com.bxi.dances/dance_part_{number}"
