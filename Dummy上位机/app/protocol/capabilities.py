"""Transport capabilities verified against firmware_new ascii_protocol.cpp.

UART4 silently returns generic ``ok`` for many absent # handlers. Block those
commands before sending so that generic acknowledgement cannot mean success.
"""
COMMON_HASH = {
    "#GETJPOS", "#GETLPOS", "#GETRAILPOS", "#STACK", "#MOTION_STATUS",
    "#GET_CMDMODE", "#CMDMODE", "#RGB_GET", "#RGB_STATUS",
    "#STALL_GET", "#STALL_SET", "#STALL_SAVE", "#STALL_STATUS",
    "#CAL_GET", "#CAL_SET", "#CAL_SAVE", "#CAL_STATUS",
}
COMMON_BANG = {"!START", "!DISABLE", "!STOP", "!HOME", "!RESET", "!CALIBRATION",
               "!HAND_O", "!HAND_C", "!HAND_I", "!HAND_POS"}


def supported(command, connection_type="USB"):
    if connection_type.upper() == "USB":
        return True
    verb = command.split()[0] if command.split() else ""
    return (command.startswith((">", "@", "&")) or verb in COMMON_HASH or
            verb in COMMON_BANG or verb.startswith("!RGB_"))


def diagnostic(command):
    import re
    return (command in ("#RGB_STATUS", "#RGB_GET", "!RGB_BRIGHT") or
            re.fullmatch(r"#STALL_STATUS\s+[1-7](?:\s+LAST)?", command) is not None or
            re.fullmatch(r"#CAL_STATUS\s+[78](?:\s+LAST)?", command) is not None)
