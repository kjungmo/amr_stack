"""Logging for the AMR stack: console + rotating file, per-module levels.

The 'amr' logger is the package root; cfg.level sets its level, so
module_levels overrides (e.g. {'amr.slam': 'DEBUG'}) act relative to it.
Handlers carry no level of their own. GUI attaches its own handler via
attach_handler().
"""
import logging
import os
from logging.handlers import RotatingFileHandler

_FORMAT = "%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s"
_DATEFMT = "%H:%M:%S"


def setup_logging(cfg) -> logging.Logger:
    root = logging.getLogger("amr")
    root.setLevel(getattr(logging, cfg.level.upper()))
    root.propagate = False
    for h in list(root.handlers):
        root.removeHandler(h)
        h.close()
    fmt = logging.Formatter(_FORMAT, _DATEFMT)
    if cfg.console:
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        root.addHandler(sh)
    if cfg.file:
        os.makedirs(os.path.dirname(cfg.file) or ".", exist_ok=True)
        fh = RotatingFileHandler(cfg.file, maxBytes=cfg.max_bytes,
                                 backupCount=cfg.backup_count)
        fh.setFormatter(fmt)
        root.addHandler(fh)
    for name, level in cfg.module_levels.items():
        logging.getLogger(name).setLevel(getattr(logging, level.upper()))
    return root


def get_logger(name: str) -> logging.Logger:
    full = name if name == "amr" or name.startswith("amr.") else "amr." + name
    return logging.getLogger(full)


def attach_handler(handler: logging.Handler) -> None:
    logging.getLogger("amr").addHandler(handler)
