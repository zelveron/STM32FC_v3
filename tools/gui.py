#!/usr/bin/env python3
"""Launch the STM32FC ground station (live USB by default; --demo for preview)."""
import sys
from gui_controller import MonitorApp, main

if __name__ == "__main__":
    sys.exit(main())
