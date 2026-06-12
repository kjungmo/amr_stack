#!/usr/bin/env bash
# Build the project venv. NOTE: this host's Ubuntu 20.04 has a broken ensurepip,
# so the venv is created --without-pip and pip is bootstrapped via get-pip.py.
# numpy/PyYAML are taken from the system (--system-site-packages); only pytest
# (and pip/setuptools/wheel) are fetched from PyPI.
set -euo pipefail
cd "$(dirname "$0")/.."
unset PYTHONPATH || true   # keep ROS Noetic's dist-packages off the path

python3 -m venv --system-site-packages --without-pip .venv

if ! .venv/bin/python -m pip --version >/dev/null 2>&1; then
  GP="${GET_PIP_PY:-/tmp/get-pip-3.8.py}"
  if [ ! -f "$GP" ]; then
    GET_PIP_PY="$GP" .venv/bin/python - <<'PY'
import os, urllib.request
dst = os.environ["GET_PIP_PY"]
urllib.request.urlretrieve("https://bootstrap.pypa.io/pip/3.8/get-pip.py", dst)
print("fetched get-pip ->", dst)
PY
  fi
  .venv/bin/python "$GP"
fi

.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -c "import amr, numpy, yaml; print('amr', amr.__version__, 'ready')"
