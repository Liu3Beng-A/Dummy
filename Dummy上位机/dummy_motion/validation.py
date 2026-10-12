"""Wire quantization is part of validation, not an afterthought."""
import numpy as np
from .frames import from_wire_joints, to_wire_joints, error


def movej_command(model, q, target, tool, speed=35):
    if not np.isfinite(speed) or not 1 <= speed <= 100:
        raise ValueError("Speed must be 1..100")
    if not model.valid(q):
        raise ValueError("Joint limit exceeded")
    def number(value):
        text = f"{value:.2f}".rstrip("0").rstrip(".")
        return "0" if text in ("-0", "") else text
    fields = [number(value) for value in to_wire_joints(q)]
    command = ">" + ",".join(fields + [number(speed)])
    sent = from_wire_joints([float(value) for value in fields])
    residual = error(model.fk(sent, tool), target)
    if (len(command.encode("ascii")) > 63 or not model.valid(sent) or
            np.linalg.norm(residual[:3]) > .0002 or np.linalg.norm(residual[3:]) > np.deg2rad(.1)):
        raise ValueError("Quantized command does not meet the pose/packet tolerance")
    return command, sent
