# CLAUDE.md - openpilot Development Guide

This document provides guidance for AI assistants working with the openpilot codebase.

## Project Overview

openpilot is an open-source autonomous driving system developed by comma.ai. It provides Adaptive Cruise Control (ACC) and Lane Keeping Assist System (LKAS) functionality for supported vehicles. The system runs on the EON Dashcam DevKit and communicates with vehicles via the panda OBD-II dongle.

**Python Version:** Python 2.7 (note the shebang `#!/usr/bin/env python2.7` in entry points)

## Repository Structure

```
openpilot/
├── apk/                    # Android APK files for UI
├── cereal/                 # Cap'n Proto message schemas (IPC)
├── common/                 # Shared utility libraries
├── installer/              # Auto-update and installation scripts
├── models/                 # Neural network model files
├── opendbc/                # CAN database files (DBC format)
├── panda/                  # Firmware for CAN interface hardware
├── phonelibs/              # Pre-built native dependencies
├── pyextra/                # Bundled Python packages
└── selfdrive/              # Main application code
    ├── assets/             # UI assets (sounds, images)
    ├── athena/             # Cloud connectivity daemon
    ├── boardd/             # CAN board communication (C++)
    ├── can/                # CAN message parsing utilities
    ├── car/                # Vehicle-specific implementations
    ├── common/             # Shared C/C++ code
    ├── controls/           # Control algorithms (main logic)
    ├── debug/              # Debugging and analysis tools
    ├── locationd/          # GPS/positioning/localization
    ├── logcatd/            # Android log daemon
    ├── loggerd/            # Data logging daemon
    ├── mapd/               # Map data and navigation
    ├── orbd/               # ORB-SLAM visual odometry
    ├── proclogd/           # Process logging
    ├── sensord/            # Sensor interface (C)
    ├── test/               # Test infrastructure
    ├── ui/                 # Qt-based user interface (C++)
    └── visiond/            # Computer vision daemon (C++)
```

## Key Entry Points

- **`selfdrive/manager.py`** - Main entry point and process orchestrator
- **`selfdrive/controls/controlsd.py`** - Main vehicle control loop
- **`selfdrive/controls/plannerd.py`** - Path planning and trajectory
- **`selfdrive/controls/radard.py`** - Radar data processing

## Build System

### Building the Project
```bash
# Build all components (runs manager.py in prepare-only mode)
make

# Or directly:
cd selfdrive && PYTHONPATH=/path/to/openpilot PREPAREONLY=1 ./manager.py
```

### Component Builds
Individual components have their own Makefiles in their directories:
- `cereal/Makefile` - Cap'n Proto message generation
- `selfdrive/boardd/Makefile` - Board communication daemon
- `selfdrive/visiond/Makefile` - Vision processing daemon
- `selfdrive/ui/Makefile` - User interface
- `selfdrive/controls/lib/lateral_mpc/Makefile` - MPC controllers

Components are built with `make -j4` by the manager when needed.

## Testing

### Running Tests
```bash
# Run all tests via Docker
./run_docker_tests.sh

# Run fingerprint tests
cd selfdrive/test && ./test_fingerprints.py

# Run longitudinal control simulation tests
cd selfdrive/test/tests/plant && OPTEST=1 ./test_longitudinal.py
```

### Test Framework
- Uses `nose` testing framework
- `@phone_only` decorator for tests requiring EON hardware
- Integration tests with `@with_processes(['controlsd', 'radard'])` decorator

### CI Pipeline (Travis CI)
1. Fingerprint consistency tests
2. Pyflakes syntax checking (excludes `pyextra/`, `panda/`)
3. Pylint code quality (excludes `pyextra/`, `panda/`)
4. Longitudinal control simulation tests

## Linting and Code Quality

### Running Linters
```bash
# Pyflakes (syntax/import checking)
pyflakes $(find . -iname "*.py" | grep -vi "^\./pyextra.*" | grep -vi "^\./panda")

# Pylint (code quality)
pylint $(find . -iname "*.py" | grep -vi "^\./pyextra.*" | grep -vi "^\./panda")
```

### Pylint Configuration
Configuration is in `.pylintrc`. Key settings:
- Parallel jobs: 4
- Many convention checks disabled (see file for full list)
- scipy is whitelisted for extension packages

## Code Style and Conventions

### Naming Conventions
- **Classes:** PascalCase (e.g., `CarInterface`, `CarState`, `VehicleModel`)
- **Functions/methods:** snake_case (e.g., `get_can_parser()`, `start_managed_process()`)
- **Constants:** UPPER_SNAKE_CASE (e.g., `BASEDIR`, `STEER_MAX`, `CAMERA_MSGS`)
- **Private functions:** Leading underscore (e.g., `_get_interface_names()`)

### Import Order
```python
# Standard library
import os
import sys
import subprocess

# Third-party packages
import zmq
import numpy as np

# Cereal (message definitions)
from cereal import car, log

# Common utilities
from common.params import Params
from common.numpy_fast import clip, interp

# Selfdrive modules
from selfdrive.services import service_list
import selfdrive.messaging as messaging
```

### Module Structure Pattern
```python
#!/usr/bin/env python2.7
# Constants at top
SOME_CONSTANT = 42

# Helper functions
def helper_function():
    pass

# Main classes
class MainClass(object):
    pass

# Entry point
def main(gctx=None):
    """Main function called by manager."""
    pass

if __name__ == "__main__":
    main()
```

## Architecture Patterns

### Process Management
The `manager.py` orchestrates all processes:
- **Persistent processes:** Always running (thermald, ui, uploader, etc.)
- **Car-started processes:** Run only when vehicle is active (controlsd, visiond, etc.)

### Inter-Process Communication
Uses ZMQ pub/sub pattern via Cap'n Proto serialization:
```python
import zmq
import selfdrive.messaging as messaging

# Create publisher
context = zmq.Context()
sock = messaging.pub_sock(context, service_list['carState'].port)

# Create subscriber
sock = messaging.sub_sock(context, service_list['thermal'].port)

# Send message
msg = messaging.new_message()
msg.init('carState')
sock.send(msg.to_bytes())

# Receive message
msg = messaging.recv_sock(sock, wait=True)
```

### Service Ports
Defined in `selfdrive/service_list.yaml`. Key services:
- `frame` [8002] - Video frames
- `can` [8006] - CAN messages
- `carState` [8021] - Vehicle state
- `carControl` [8023] - Control commands
- `plan` [8024] - Planning output
- `thermal` [8005] - Thermal data

## Car Interface Pattern

Each supported manufacturer has a directory in `selfdrive/car/` with:

### values.py
```python
class CAR:
    MODEL_NAME = "MAKE MODEL YEAR TRIM"

FINGERPRINTS = {
    CAR.MODEL_NAME: [{0x158: 8, 0x192: 5, ...}],  # CAN message IDs
}

DBC = {
    CAR.MODEL_NAME: 'honda_civic_touring_2016_can_generated',
}
```

### interface.py
```python
class CarInterface(object):
    def __init__(self, CP, sendcan=None):
        self.CP = CP
        self.CS = CarState(CP)
        # ...

    @staticmethod
    def get_params(candidate, fingerprint):
        ret = car.CarParams.new_message()
        # Configure vehicle parameters
        return ret

    def update(self, c):
        # Read CAN, return car.CarState
        return ret.as_reader()

    def apply(self, c):
        # Apply car.CarControl commands
        pass
```

### carstate.py
Parses incoming CAN messages into vehicle state.

### carcontroller.py
Generates CAN messages for vehicle control.

## Environment Variables

- `BASEDIR` - Root directory of openpilot
- `DONGLE_ID` - Device identifier
- `PASSIVE` - Chffrplus passive mode
- `PREPAREONLY` - Build without running
- `NOBOARD` - Skip panda connection
- `NOLOG` - Disable logging
- `NOUPLOAD` - Disable upload
- `NOVISION` - Disable vision
- `LEAN` - Minimal process set
- `NOCONTROL` - Disable control processes
- `CLEAN` - Set when running clean (non-dirty) code
- `OPTEST` - Enable integration test mode

## Safety Considerations

This is safety-critical automotive software. When making changes:

1. **Never bypass safety limits** defined in panda firmware and control code
2. **Preserve disengagement triggers** (pedal press, cancel button)
3. **Maintain actuator constraints** (steering torque, brake limits)
4. **Test thoroughly** before deploying to vehicles
5. **Review SAFETY.md** for manufacturer-specific safety implementations

## Common Development Tasks

### Adding Support for a New Car Model
1. Identify CAN messages using panda and cabana
2. Add fingerprint to `selfdrive/car/<make>/values.py`
3. Implement/extend CarState for message parsing
4. Configure CarParams in interface.py
5. Test with `test_fingerprints.py`

### Modifying Control Behavior
1. Changes typically go in `selfdrive/controls/`
2. `controlsd.py` - Main control loop
3. `lib/planner.py` - Path planning
4. `lib/longcontrol.py` - Longitudinal control
5. `lib/latcontrol.py` - Lateral control

### Debugging
- Use tools in `selfdrive/debug/`
- Check logs via `selfdrive/logmessaged.py`
- Monitor processes with thermal data

## Important Files Reference

| File | Purpose |
|------|---------|
| `selfdrive/manager.py` | Process orchestration |
| `selfdrive/controls/controlsd.py` | Main control daemon |
| `selfdrive/messaging.py` | ZMQ messaging helpers |
| `selfdrive/services.py` | Service port definitions |
| `selfdrive/config.py` | Conversion constants |
| `cereal/log.capnp` | Message schema definitions |
| `cereal/car.capnp` | Car-specific message schemas |
| `common/params.py` | Parameter storage |
| `.pylintrc` | Linting configuration |
| `.travis.yml` | CI configuration |

## Quick Reference Commands

```bash
# Build everything
make

# Run linting
pylint selfdrive/

# Run tests in Docker
./run_docker_tests.sh

# Check fingerprints
cd selfdrive/test && python test_fingerprints.py
```
