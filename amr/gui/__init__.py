"""tkinter HRI console for the AMR stack.

Importing :mod:`amr.gui` is cheap and does *not* import tkinter; the GUI
modules (:mod:`amr.gui.app`, :mod:`amr.gui.map_canvas`, :mod:`amr.gui.panels`)
import Tk lazily when they are first imported. The CLI imports
``amr.gui.app`` only for the ``gui`` subcommand, so the headless commands keep
working even on a build without Tk.
"""
