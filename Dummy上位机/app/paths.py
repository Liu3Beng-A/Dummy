"""Application resource paths, independent of the working directory."""
from pathlib import Path
import sys


def application_root():
    if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
        return Path(sys._MEIPASS)
    return Path(__file__).resolve().parent.parent


def resource_path(*parts):
    return application_root() / "resources" / Path(*parts)
