"""Verify an installed wheel outside the source checkout; never opens hardware."""

import importlib
import importlib.metadata
import importlib.resources
import math
from pathlib import Path

from linkerbot_arm.arm.kinetix import ArmKinetix

module = importlib.import_module("linkerbot_arm")
source = Path(__file__).resolve().parents[1]
assert not Path(module.__file__).resolve().is_relative_to(source), module.__file__
assert importlib.resources.files(module).joinpath("py.typed").is_file()
distribution = importlib.metadata.distribution("linkerbot-arm")
assert any("LICENSE" in str(path) for path in distribution.files or ()), (
    "Missing LICENSE"
)
print(
    f"Installed {distribution.metadata['Name']} {distribution.version}: {module.__file__}"
)

for model in ("a7", "a7_lite", "p7", "h75"):
    for side in ("left", "right"):
        pose = ArmKinetix.from_builtin(model, side).forward_kinematics([0.0] * 7)
        assert all(math.isfinite(value) for value in pose.to_list()), (model, side)
print("All eight installed URDF models loaded and produced finite FK")
