"""Pure gamepad configuration/input; SDL acquisition is an explicit opt-in."""
from .config import default_config, validate_config
from .input import decode_gamepad, motion_delta

__all__ = ["default_config", "validate_config", "decode_gamepad", "motion_delta"]
