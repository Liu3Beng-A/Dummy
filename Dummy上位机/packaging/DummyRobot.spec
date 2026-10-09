# Build from this directory using: python -m PyInstaller DummyRobot.spec
from pathlib import Path

project = Path(SPECPATH).parent
analysis = Analysis(
    [str(project / "main.py")], pathex=[str(project)],
    binaries=[], datas=[(str(project / "resources"), "resources")],
    hiddenimports=["PySide6.QtWebEngineWidgets", "PySide6.QtWebChannel"],
    hookspath=[], hooksconfig={}, runtime_hooks=[], excludes=["tkinter"],
    noarchive=False,
)
archive = PYZ(analysis.pure)
executable = EXE(archive, analysis.scripts, [], exclude_binaries=True,
                 name="DummyRobot", debug=False, bootloader_ignore_signals=False,
                 strip=False, upx=False, console=False,
                 icon=str(project / "resources" / "icons" / "dummy.ico"),
                 version=str(project / "packaging" / "version_info.txt"))
bundle = COLLECT(executable, analysis.binaries, analysis.datas,
                 strip=False, upx=False, name="DummyRobot")
