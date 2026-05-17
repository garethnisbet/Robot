#!/usr/bin/env python3
"""
Install Python API dependencies without requiring sudo.

Installs packages into the user's local site-packages (~/.local/lib/...).
Run with:  python3 install_dependencies.py
"""

import subprocess
import sys


PACKAGES = [
    "websockets",
    "ipython",
    "numpy",
    "trimesh",
    "rtree",
]


def main():
    pip = [sys.executable, "-m", "pip", "install", "--user"]

    print(f"Using Python: {sys.executable}")
    print(f"Installing: {', '.join(PACKAGES)}\n")

    result = subprocess.run(pip + PACKAGES)

    if result.returncode != 0:
        print("\nInstallation failed.", file=sys.stderr)
        sys.exit(result.returncode)

    print("\nAll dependencies installed.")
    print("If this is your first install, you may need to add ~/.local/bin to PATH:")
    print("  export PATH=\"$HOME/.local/bin:$PATH\"")


if __name__ == "__main__":
    main()
