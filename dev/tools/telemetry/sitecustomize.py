"""Dev-only stdout/stderr tee for Webots controllers on Windows.

Webots on Windows is a GUI-subsystem app: `--stdout` output never reaches a
redirected file, so batch runs had no controller telemetry. This module is
copied into the DEV venv's site-packages (dev/tools/setup_env.ps1). When
SAR_TEE_DIR is set (run_batch.py sets it per run), every Python process that
venv starts also writes its output to SAR_TEE_DIR/<cwd-name>_<pid>.log. The
controller and supervisor sources are untouched; with SAR_TEE_DIR unset this
does nothing at all.
"""
import os as _os
import sys as _sys

_dir = _os.environ.get("SAR_TEE_DIR")
if _dir:
    try:
        _os.makedirs(_dir, exist_ok=True)
        _name = _os.path.basename(_os.getcwd()) or "python"
        _fh = open(_os.path.join(_dir, f"{_name}_{_os.getpid()}.log"), "a",
                   encoding="utf-8", errors="replace", buffering=1)

        class _Tee:
            def __init__(self, inner):
                self._inner = inner

            def write(self, s):
                try:
                    _fh.write(s)
                except Exception:
                    pass
                return self._inner.write(s) if self._inner else len(s)

            def flush(self):
                try:
                    _fh.flush()
                except Exception:
                    pass
                if self._inner:
                    self._inner.flush()

            def __getattr__(self, name):
                return getattr(self._inner, name)

        _sys.stdout = _Tee(_sys.stdout)
        _sys.stderr = _Tee(_sys.stderr)
    except Exception:
        pass
