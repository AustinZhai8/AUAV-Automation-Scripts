# FS-i6 -> ArduPilot RC connection check

This small script checks whether your FlySky FS-i6 (via its receiver) is providing RC input to an ArduPilot vehicle exposed over MAVLink.

Usage

1. Install dependencies:

```bash
pip install -r requirements.txt
```

2. Run the check (example for Windows COM port):

```bash
python check_fs_i6.py --port COM3 --baud 57600
```

Or connect to a MAVLink TCP endpoint created by Mission Planner or SITL:

```bash
python check_fs_i6.py --port tcp:127.0.0.1:5760
```

Exit codes

- `0`: RC input detected
- `1`: No RC input detected within timeout
- `2`: Could not connect to vehicle / no heartbeat
# AUAV-Automation-Scripts