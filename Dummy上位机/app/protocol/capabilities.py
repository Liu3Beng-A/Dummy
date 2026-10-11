"""Transport capabilities verified against firmware_new ascii_protocol.cpp.

UART4 silently returns generic ``ok`` for many absent # handlers. Block those
commands before sending so that generic acknowledgement cannot mean success.
"""
import re


COMMON_HASH = {
    "#GETJPOS", "#GETLPOS", "#GETRAILPOS", "#STACK", "#MOTION_STATUS",
    "#GET_CMDMODE", "#CMDMODE", "#RGB_GET", "#RGB_STATUS",
    "#STALL_GET", "#STALL_SET", "#STALL_SAVE", "#STALL_STATUS",
    "#CAL_GET", "#CAL_SET", "#CAL_SAVE", "#CAL_STATUS",
    "#FW_VERSION", "#CAN_DIAG",
}
COMMON_BANG = {"!START", "!DISABLE", "!STOP", "!HOME", "!RESET", "!CALIBRATION",
               "!HAND_O", "!HAND_C", "!HAND_I", "!HAND_POS"}


def supported(command, connection_type="USB"):
    if connection_type.upper() == "USB":
        return True
    verb = command.split()[0] if command.split() else ""
    return (command.startswith((">", "@", "&")) or verb in COMMON_HASH or
            verb in COMMON_BANG or verb.startswith("!RGB_"))


def identity_query(command):
    """Strict single-node query, with an optional nonzero uint16 request id."""
    match = re.fullmatch(r"(#FW_VERSION|#CAN_DIAG)[ \t]+([0-8])(?:[ \t]+([0-9]{1,5}))?", command)
    return bool(match and (match[1] != "#CAN_DIAG" or match[2] != "0") and
                (match[3] is None or 1 <= int(match[3]) <= 65535))


def diagnostic(command):
    return (command in ("#RGB_STATUS", "#RGB_GET", "!RGB_BRIGHT", "#MOTION_STATUS") or
            identity_query(command) or
            re.fullmatch(r"#STALL_STATUS\s+[1-7](?:\s+LAST)?", command) is not None or
            re.fullmatch(r"#CAL_STATUS\s+[78](?:\s+LAST)?", command) is not None)
