PY  := .venv/bin/python
AMR := .venv/bin/amr

# Keep ROS Noetic's PYTHONPATH out of every recipe for a deterministic env.
unexport PYTHONPATH

.PHONY: setup test test-fast demo gui clean

setup:
	bash scripts/setup.sh
test:
	$(PY) -m pytest -q
test-fast:
	$(PY) -m pytest -q -m "not slow"
demo:
	$(AMR) demo
gui:
	$(AMR) gui
clean:
	rm -rf .venv build dist *.egg-info logs $$(find . -name __pycache__)
