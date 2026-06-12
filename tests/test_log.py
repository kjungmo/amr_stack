import logging

from amr.core.config import LoggingConfig
from amr.core.log import get_logger, setup_logging


def _read(p):
    return p.read_text() if p.exists() else ""


def test_file_and_levels(tmp_path):
    cfg = LoggingConfig(level="INFO", file=str(tmp_path / "amr.log"),
                        console=False, module_levels={"amr.slam": "DEBUG"})
    setup_logging(cfg)
    get_logger("slam").debug("slam-debug-visible")
    get_logger("nav").debug("nav-debug-hidden")
    get_logger("nav").info("nav-info-visible")
    for h in logging.getLogger("amr").handlers:
        h.flush()
    text = _read(tmp_path / "amr.log")
    assert "slam-debug-visible" in text
    assert "nav-debug-hidden" not in text
    assert "nav-info-visible" in text


def test_idempotent_setup(tmp_path):
    cfg = LoggingConfig(file=str(tmp_path / "a.log"), console=False)
    setup_logging(cfg)
    setup_logging(cfg)
    assert len(logging.getLogger("amr").handlers) == 1
