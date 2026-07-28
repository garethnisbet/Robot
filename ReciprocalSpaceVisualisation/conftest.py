"""Root conftest — puts the project root on sys.path so `tests/` can import the
flat modules (`geometry`) the same way `server.py` does."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
