import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


@pytest.fixture
def config_path():
    """Absolute path to a device config in the repo root."""
    return lambda name: str(ROOT / f"{name}_config.json")
