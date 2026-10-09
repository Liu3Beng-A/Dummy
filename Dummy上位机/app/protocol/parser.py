"""ASCII framing and strict finite pose parsing."""
import math
import codecs


class LineFramer:
    def __init__(self, maximum=8192):
        self.fragment = ""
        self.maximum = maximum
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")

    def feed(self, data):
        text = self.decoder.decode(data) if isinstance(data, bytes) else data
        rows = (self.fragment + text).split("\n")
        self.fragment = rows.pop()
        if len(self.fragment) > self.maximum:
            # Overflow is invalid framing, never infer an acknowledgement.
            self.fragment = ""
            raise ValueError("串口回复超过最大行长度，已丢弃不完整行")
        return [row.strip() for row in rows if row.strip()]


def finite_numbers(payload, count=None):
    try:
        values = [float(token) for token in payload.replace(",", " ").split()]
    except (TypeError, ValueError):
        return None
    if any(not math.isfinite(value) for value in values):
        return None
    return values if count is None or len(values) == count else None
