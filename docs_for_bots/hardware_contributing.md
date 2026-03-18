# DimOS Hardware Contributing Guide

## 1. What is DimOS?

DimOS is a modular robotics framework for composing sensor drivers, control loops, navigation planners, and AI agents into deployable systems. It provides:

- **Typed pub/sub streams** (LCM multicast by default) connecting modules
- **RPC** for remote method calls between modules
- **Blueprints** for declarative system composition
- **Worker process isolation** for fault tolerance
- **Docker module support** for containerized components

Communication happens over LCM multicast on localhost. No ROS installation is required.

---

## 2. Core Concepts: Modules, Streams, and Blueprints

### Module

A Module is a unit of computation. It declares typed input/output streams and RPC methods:

```python
from dimos.core.module import Module, ModuleConfig
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

class MyCameraConfig(ModuleConfig):
    device_id: int = 0
    fps: float = 30.0

class MyCameraModule(Module[MyCameraConfig]):
    default_config = MyCameraConfig

    # Outputs (this module publishes these)
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    # Inputs (this module subscribes to these)
    # trigger: In[SomeTriggerType]

    @rpc
    def start(self) -> None:
        # Initialize hardware, start publishing
        ...

    @rpc
    def stop(self) -> None:
        # Release hardware
        super().stop()
```

### In[T] and Out[T]

- `Out[T]` - Output stream. Call `self.my_output.publish(value)` to send data.
- `In[T]` - Input stream. Call `self.my_input.subscribe(callback)` to receive data, or `self.my_input.observable()` for RxPY operators.

Streams are declared as **class-level type annotations**. `Module.__init__` automatically instantiates them.

```python
# Publishing
self.color_image.publish(image)

# Subscribing (simple callback)
self.cmd_vel.subscribe(self._on_twist)

# Subscribing (RxPY pipeline)
self.sensor_data.observable().pipe(
    ops.throttle_first(0.1),
    ops.map(self._process),
).subscribe(self._on_result)

# Get latest value on demand
getter = self.my_input.hot_latest()
latest_value = getter()

# Block until next value
value = self.my_input.get_next(timeout=10.0)
```

### Blueprint

A Blueprint is an immutable description of which modules to run and how to wire them. Streams with the **same name and type** (one `In`, one `Out`) auto-connect.

```python
from dimos.core.blueprints import autoconnect

system = autoconnect(
    MyCameraModule.blueprint(device_id=0),
    MyProcessorModule.blueprint(),
).global_config(n_workers=2)

coordinator = system.build()
coordinator.loop()  # Blocks until Ctrl+C
```

Every Module class has a `.blueprint` class property that returns a partial constructor:

```python
# These are equivalent:
MyCameraModule.blueprint(device_id=0)
# Produces a Blueprint containing one _BlueprintAtom for MyCameraModule
```

---

## 3. Running and Testing Blueprints

### From a standalone repo

```bash
# Create project
mkdir my-hardware && cd my-hardware
git init

# Setup Python environment (requires Python 3.12, uv)
uv init --python 3.12
uv add "dimos @ git+https://github.com/dimensionalOS/dimos.git@dev"
uv sync

# Run a blueprint
uv run python -c "
from dimos.core.blueprints import autoconnect
from my_module import MyModule

autoconnect(MyModule.blueprint()).build().loop()
"
```

### Testing a blueprint programmatically

```python
import time

coordinator = autoconnect(
    MyModule.blueprint(some_param=42),
).build()

# Let it run briefly
time.sleep(5)
coordinator.stop()
```

---

## 4. Available Stream Types

All types live in `dimos/msgs/` and mirror ROS message conventions. Each type has `lcm_encode()` / `lcm_decode()` methods for transport serialization.

### sensor_msgs

#### `Image`
Camera frame data. Similar to ROS `sensor_msgs/Image`.

| Field | Type | Description |
|-------|------|-------------|
| `data` | `np.ndarray` | Pixel data (HxW or HxWxC) |
| `format` | `ImageFormat` | `BGR`, `RGB`, `RGBA`, `GRAY`, `DEPTH_MM`, `DEPTH_M`, etc. |
| `frame_id` | `str` | TF frame name |
| `ts` | `float` | Timestamp (seconds) |

Conversion helpers: `.to_rgb()`, `.to_bgr()`, `.to_grayscale()`, `.to_jpeg()`, `.from_jpeg()`, `.to_pil()`, `.from_pil()`, `.to_base64()`.

When publishing `Image`, consumers often also need `CameraInfo` on a paired stream for calibration data.

#### `CameraInfo`
Camera intrinsic calibration. Similar to ROS `sensor_msgs/CameraInfo`.

| Field | Type | Description |
|-------|------|-------------|
| `height`, `width` | `int` | Image dimensions |
| `K` | `list[float]` | 3x3 intrinsic matrix (row-major) |
| `D` | `list[float]` | Distortion coefficients |
| `R` | `list[float]` | 3x3 rectification matrix |
| `P` | `list[float]` | 3x4 projection matrix |
| `distortion_model` | `str` | e.g. `"plumb_bob"`, `"rational_polynomial"` |
| `frame_id` | `str` | TF frame |
| `ts` | `float` | Timestamp |

#### `JointState`
Joint positions/velocities/efforts. Similar to ROS `sensor_msgs/JointState`.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `list[str]` | Joint names |
| `position` | `list[float]` | Joint positions (rad or m) |
| `velocity` | `list[float]` | Joint velocities |
| `effort` | `list[float]` | Joint efforts (torques/forces) |
| `frame_id` | `str` | TF frame |
| `ts` | `float` | Timestamp |

#### `PointCloud2`
3D point cloud. Similar to ROS `sensor_msgs/PointCloud2`. Uses Open3D tensors internally.

| Field | Type | Description |
|-------|------|-------------|
| `_pcd_tensor` | Open3D TensorPointCloud | Point data |
| `frame_id` | `str` | TF frame |
| `ts` | `float` | Timestamp |

Helper: `PointCloud2.from_numpy(points, colors=None)`.

#### `Imu`
IMU sensor data. Similar to ROS `sensor_msgs/Imu`.

| Field | Type | Description |
|-------|------|-------------|
| `orientation` | `Quaternion` | Orientation estimate |
| `angular_velocity` | `Vector3` | Gyroscope (rad/s) |
| `linear_acceleration` | `Vector3` | Accelerometer (m/s^2) |
| `orientation_covariance` | `list[float]` | 3x3 covariance |
| `angular_velocity_covariance` | `list[float]` | 3x3 covariance |
| `linear_acceleration_covariance` | `list[float]` | 3x3 covariance |
| `frame_id` | `str` | TF frame |
| `ts` | `float` | Timestamp |

### geometry_msgs

#### `Twist`
Linear and angular velocity. Similar to ROS `geometry_msgs/Twist`.

| Field | Type | Description |
|-------|------|-------------|
| `linear` | `Vector3` | Linear velocity (m/s) |
| `angular` | `Vector3` | Angular velocity (rad/s) |

Supports arithmetic: `twist1 + twist2`, `twist * scalar`, `Twist.zero()`.

#### `Vector3`
3D vector. Fields: `x`, `y`, `z` (all `float`). Supports `+`, `-`, `*`, `abs()`.

#### `Quaternion`
Unit quaternion. Fields: `x`, `y`, `z`, `w`. Helpers: `.from_euler(roll, pitch, yaw)`, `.to_euler()`.

#### `Pose` / `PoseStamped`
Position + orientation. `Pose` has `position: Vector3` and `orientation: Quaternion`. `PoseStamped` adds `frame_id` and `ts`.

#### `Transform` / `TransformStamped`
Homogeneous transform between frames. Fields: `translation: Vector3`, `rotation: Quaternion`. `TransformStamped` adds `frame_id`, `child_frame_id`, `ts`.

### nav_msgs

#### `Odometry`
Robot odometry. Similar to ROS `nav_msgs/Odometry`.

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `PoseWithCovariance` | Position + orientation + covariance |
| `twist` | `TwistWithCovariance` | Velocity + covariance |
| `frame_id` | `str` | Parent frame (e.g. `"odom"`) |
| `child_frame_id` | `str` | Robot frame (e.g. `"base_link"`) |
| `ts` | `float` | Timestamp |

Convenience properties: `.x`, `.y`, `.z`, `.vx`, `.vy`, `.vz`, `.position`, `.orientation`.

---

## 5. Formatting and Testing Tools

### Project setup (standalone repo)

```bash
uv init --python 3.12
uv add "dimos @ git+https://github.com/dimensionalOS/dimos.git@dev"
uv add --dev ruff mypy pytest
```

### pyproject.toml

```toml
[project]
name = "my-dimos-hardware"
version = "0.1.0"
requires-python = ">=3.12"
dependencies = [
    "dimos @ git+https://github.com/dimensionalOS/dimos.git@dev",
]

[tool.ruff]
line-length = 120

[tool.ruff.format]
quote-style = "double"

[tool.mypy]
python_version = "3.12"
warn_return_any = true
warn_unused_configs = true

[tool.pytest.ini_options]
testpaths = ["run/tests"]
timeout = 30
```

### Formatting and linting

```bash
# Format code
uv run ruff format .

# Lint (with auto-fix)
uv run ruff check --fix .

# Type check
uv run mypy --python-version 3.12 .
```

### Running tests

```bash
# Automated tests (CI-friendly, no hardware needed)
./run/tests/ -v

# Human testing with real hardware
./run/trial
```

---

## 6. Native Modules

Native modules wrap C/C++ executables as blueprint-compatible modules. The Python wrapper declares In/Out ports for wiring; the actual work happens in the subprocess.

### How they work

1. Python wrapper declares In/Out ports (type annotations)
2. On `start()`, the blueprint resolves LCM topics for each port
3. The native binary is launched with CLI args: `./binary --port_name /lcm/topic ...`
4. The binary does pub/sub directly on LCM multicast
5. On `stop()`, the binary receives SIGTERM

### Directory structure of a native module

```
my_sensor/
    module.py                   # Python wrapper (In/Out ports, config)
    blueprints/
        __init__.py
        basic.py
    cpp/
        main.cpp                # C++ binary source
        CMakeLists.txt          # CMake build
        flake.nix               # Nix flake for hermetic builds
    common/                     # (optional) shared headers
        dimos_native_module.hpp # CLI arg parser (copy from dimos)
```

### Step 1: Python wrapper (module.py)

```python
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sensor_msgs.Imu import Imu

class MySensorConfig(NativeModuleConfig):
    executable: str = "cpp/result/bin/my_sensor"
    build_command: str | None = "nix build .#my_sensor"
    cwd: str | None = "cpp"
    rebuild_on_change: list[str] = ["cpp/main.cpp", "cpp/CMakeLists.txt"]  # auto-rebuild when these change (future feature)

    # Hardware config — these become CLI args automatically
    host_ip: str = "192.168.1.5"
    sensor_ip: str = "192.168.1.100"
    frequency: float = 10.0
    frame_id: str = "sensor_link"

class MySensorModule(NativeModule[MySensorConfig]):
    default_config = MySensorConfig
    pointcloud: Out[PointCloud2]
    imu: Out[Imu]
```

When `start()` is called, the binary receives:
```
./my_sensor --pointcloud "/rpc/.../pointcloud#sensor_msgs.PointCloud2" \
            --imu "/rpc/.../imu#sensor_msgs.Imu" \
            --host_ip 192.168.1.5 --sensor_ip 192.168.1.100 \
            --frequency 10.0 --frame_id sensor_link
```

Config-to-CLI mapping rules for fields on your config subclass (excluding `NativeModuleConfig` base fields):
- `None` values are skipped
- `bool` becomes `--field true` or `--field false`
- `list` becomes `--field comma,separated,values`
- Everything else becomes `--field str(value)`
- Use `cli_exclude` to skip fields: `cli_exclude: frozenset[str] = frozenset({"internal_field"})`

### Step 2: C++ binary (cpp/main.cpp)

The binary parses CLI args for LCM topic names and config values, then does LCM pub/sub directly.

Use the `dimos::NativeModule` helper header (copy from `dimos/hardware/sensors/lidar/common/dimos_native_module.hpp`) for CLI parsing:

```cpp
#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>

// dimos-lcm generated C++ message types
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/Imu.hpp"
#include "std_msgs/Header.hpp"

// Header-only CLI parser from dimos
#include "dimos_native_module.hpp"

static std::atomic<bool> g_running{true};

static void signal_handler(int) { g_running.store(false); }

// Build a stamped header with frame_id and timestamp
static std_msgs::Header make_header(const std::string& frame_id, double ts) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header h;
    h.seq = seq.fetch_add(1, std::memory_order_relaxed);
    h.stamp.sec = static_cast<int32_t>(ts);
    h.stamp.nsec = static_cast<int32_t>((ts - h.stamp.sec) * 1e9);
    h.frame_id = frame_id;
    return h;
}

int main(int argc, char** argv) {
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    // Parse CLI args passed by NativeModule
    dimos::NativeModule mod(argc, argv);

    // Required: LCM topics for declared ports
    std::string pc_topic = mod.topic("pointcloud");   // throws if missing
    std::string imu_topic = mod.has("imu") ? mod.topic("imu") : "";

    // Config args (auto-converted from Python config fields)
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string sensor_ip = mod.arg("sensor_ip", "192.168.1.100");
    float frequency = mod.arg_float("frequency", 10.0f);
    std::string frame_id = mod.arg("frame_id", "sensor_link");

    // Init LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }

    // TODO: Initialize your hardware SDK here
    // my_sdk::Device device(sensor_ip, host_ip);
    // device.start();

    printf("[my_sensor] Running at %.1f Hz, publishing to %s\n",
           frequency, pc_topic.c_str());

    // Main loop with rate control
    auto interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / frequency));
    auto last_emit = std::chrono::steady_clock::now();

    while (g_running.load()) {
        lcm.handleTimeout(10);  // Process any incoming LCM (10ms timeout)

        auto now = std::chrono::steady_clock::now();
        if (now - last_emit >= interval) {
            double ts = std::chrono::duration<double>(
                now.time_since_epoch()).count();

            // Build and publish PointCloud2
            sensor_msgs::PointCloud2 pc;
            pc.header = make_header(frame_id, ts);
            pc.height = 1;
            // ... fill in point data from hardware ...
            lcm.publish(pc_topic, &pc);

            // Optionally publish IMU
            if (!imu_topic.empty()) {
                sensor_msgs::Imu imu_msg;
                imu_msg.header = make_header(frame_id, ts);
                // ... fill in IMU data ...
                lcm.publish(imu_topic, &imu_msg);
            }

            last_emit = now;
        }
    }

    // Cleanup
    // device.stop();
    printf("[my_sensor] Shutdown complete.\n");
    return 0;
}
```

Key C++ patterns:
- `dimos::NativeModule mod(argc, argv)` parses all `--key value` args
- `mod.topic("port_name")` gets the LCM topic string for a declared port (throws if missing)
- `mod.arg("key", "default")`, `mod.arg_float(...)`, `mod.arg_int(...)` for config values
- `lcm.publish(topic, &msg)` to send, `lcm.handleTimeout(ms)` in the main loop
- Use `std::atomic<bool>` + signal handlers for clean SIGTERM shutdown

### Step 3: CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.14)
project(my_sensor_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# Fetch dimos-lcm for C++ message headers (PointCloud2, Imu, etc.)
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# Find LCM via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Find your hardware SDK
# find_library(MY_SDK my_sdk_shared)

add_executable(my_sensor main.cpp)

target_include_directories(my_sensor PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../common    # dimos_native_module.hpp
)

target_link_libraries(my_sensor PRIVATE
  ${LCM_LIBRARIES}
  # ${MY_SDK}
)

target_link_directories(my_sensor PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS my_sensor DESTINATION bin)
```

### Step 4: flake.nix (hermetic build)

```nix
{
  description = "My sensor native module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # If your sensor needs a vendor SDK, build it here:
        # my-sdk = pkgs.stdenv.mkDerivation { ... };

        my_sensor = pkgs.stdenv.mkDerivation {
          pname = "my_sensor";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            pkgs.lcm
            pkgs.glib
            # my-sdk
          ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];
        };
      in {
        packages = {
          default = my_sensor;
          inherit my_sensor;
        };
      });
}
```

### Building and testing

```bash
# Build with nix (hermetic, recommended)
cd cpp && nix build

# Or build with cmake directly
cd cpp && mkdir -p build && cd build && cmake .. && make && make install

# Test the binary directly (no Python needed)
./result/bin/my_sensor --pointcloud /test/pc --frequency 5.0

# Test via blueprint
uv run python -c "
from dimos.core.blueprints import autoconnect
from module import MySensorModule
autoconnect(MySensorModule.blueprint()).build().loop()
"
```

### Refreshing stale binaries

If you rebuild the binary, just restart the blueprint. `NativeModule.start()` launches a fresh subprocess each time. If `build_command` is set in the config, it runs automatically before launch — so `nix build` happens on every `start()` (nix is a no-op if nothing changed).

---

## 7. Creating a Hardware Sensor/Effector Package

### Repository structure

```
my-sensor/
    pyproject.toml
    README.md
    module.py
    blueprints/
        __init__.py
        basic.py
    run/
        tests         # Automated CI-friendly tests (executable script)
        trial          # Human testing with real hardware (executable script)
```

### Step 1: Project setup

```bash
mkdir my-sensor && cd my-sensor
git init
uv init --python 3.12
uv add "dimos @ git+https://github.com/dimensionalOS/dimos.git@dev"
uv sync
```

Your `pyproject.toml`:

```toml
[project]
name = "my-sensor"
version = "0.1.0"
requires-python = ">=3.12"
dependencies = [
    "dimos @ git+https://github.com/dimensionalOS/dimos.git@dev",
    # Add hardware-specific deps here:
    # "pyserial>=3.5",
]

[project.optional-dependencies]
dev = ["ruff", "mypy", "pytest"]

[tool.ruff]
line-length = 120

[tool.mypy]
python_version = "3.12"

[tool.pytest.ini_options]
testpaths = ["run/tests"]
timeout = 30
```

### Step 2: Write module.py

```python
"""DimOS module for the XYZ Depth Camera."""

import time
from pydantic import Field
from dimos.core.module import Module, ModuleConfig
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

import numpy as np


class XYZCameraConfig(ModuleConfig):
    serial_number: str = ""
    fps: float = 30.0
    width: int = 640
    height: int = 480


class XYZCameraModule(Module[XYZCameraConfig]):
    default_config = XYZCameraConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]

    @rpc
    def start(self) -> None:
        # Initialize your hardware SDK here
        # self._device = xyz_sdk.open(self.config.serial_number)
        # self._device.set_resolution(self.config.width, self.config.height)
        # self._device.start_streaming()

        # Subscribe to hardware frames and republish
        import reactivex as rx
        self._disposables.add(
            rx.interval(1.0 / self.config.fps).subscribe(lambda _: self._capture())
        )

    def _capture(self) -> None:
        # color_frame, depth_frame = self._device.get_frames()
        color_frame = np.zeros((self.config.height, self.config.width, 3), dtype=np.uint8)
        depth_frame = np.zeros((self.config.height, self.config.width), dtype=np.uint16)

        self.color_image.publish(Image(
            data=color_frame, format=ImageFormat.BGR,
            frame_id="xyz_camera", ts=time.time(),
        ))
        self.depth_image.publish(Image(
            data=depth_frame, format=ImageFormat.DEPTH_MM,
            frame_id="xyz_camera", ts=time.time(),
        ))

    @rpc
    def stop(self) -> None:
        # self._device.stop_streaming()
        super().stop()
```

### Step 3: Create blueprints/

```python
# blueprints/basic.py
from dimos.core.blueprints import autoconnect
from my_sensor.module import XYZCameraModule

xyz_camera_basic = autoconnect(
    XYZCameraModule.blueprint(),
)
```

### Step 4: Write a README

The README should include:

```markdown
# XYZ Depth Camera Module

DimOS module for the XYZ Depth Camera.

## Supported Platforms

- Ubuntu 22.04+ (amd64)
- Ubuntu 24.04+ (arm64)

## System Dependencies

    sudo apt-get install -y libusb-1.0-0-dev libudev-dev
    # Install vendor SDK:
    # wget https://example.com/xyz-sdk-1.2.deb && sudo dpkg -i xyz-sdk-1.2.deb

## Python Dependencies

    uv add "dimos @ git+https://github.com/dimensionalOS/dimos.git@dev"
    uv add pyxyz-sdk>=1.2

## Streams

| Name | Direction | Type | Description |
|------|-----------|------|-------------|
| `color_image` | Out | `Image` (BGR) | Color frames at configured FPS |
| `depth_image` | Out | `Image` (DEPTH_MM) | Depth frames (millimeters) |
| `camera_info` | Out | `CameraInfo` | Intrinsic calibration |

## Usage

    from dimos.core.blueprints import autoconnect
    from my_sensor.module import XYZCameraModule

    autoconnect(
        XYZCameraModule.blueprint(serial_number="ABC123", fps=15),
    ).build().loop()

## Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_number` | `str` | `""` | Camera serial (empty = first found) |
| `fps` | `float` | `30.0` | Capture frame rate |
| `width` | `int` | `640` | Image width |
| `height` | `int` | `480` | Image height |
```

### Step 5: Create run/ scripts

**`run/tests`** (CI-friendly, no hardware required):

```bash
#!/usr/bin/env bash
set -euo pipefail

echo "=== Lint ==="
uv run ruff check .
uv run ruff format --check .

echo "=== Type check ==="
uv run mypy --python-version 3.12 module.py blueprints/

echo "=== Unit tests ==="
uv run pytest run/tests/ -v

echo "=== Import smoke test ==="
uv run python -c "from module import XYZCameraModule; print('Import OK')"

echo "All checks passed."
```

```bash
chmod +x run/tests
```

Put pytest-compatible test files in `run/tests/`:

```python
# run/tests/test_module.py
from module import XYZCameraModule, XYZCameraConfig


def test_config_defaults() -> None:
    cfg = XYZCameraConfig()
    assert cfg.fps == 30.0
    assert cfg.width == 640


def test_module_class_has_streams() -> None:
    hints = XYZCameraModule.__annotations__
    assert "color_image" in hints
    assert "depth_image" in hints
    assert "camera_info" in hints
```

**`run/trial`** (human testing with real hardware):

```bash
#!/usr/bin/env bash
set -euo pipefail

echo "Starting XYZ Camera trial (Ctrl+C to stop)..."
echo "Streams will be published on LCM multicast."
echo ""

uv run python -c "
from dimos.core.blueprints import autoconnect
from module import XYZCameraModule

print('Building blueprint...')
coord = autoconnect(
    XYZCameraModule.blueprint(fps=10),
).build()
print('Running. Press Ctrl+C to stop.')
coord.loop()
"
```

```bash
chmod +x run/trial
```

### Checklist

Before submitting:
- [ ] README lists all supported OS versions
- [ ] README lists all system dependencies (`apt-get`)
- [ ] README has a streams table
- [ ] README has a configuration table
- [ ] README has a usage example
- [ ] `ruff format --check .` passes
- [ ] `mypy --python-version 3.12` passes
- [ ] `run/tests` passes with no hardware attached
- [ ] `run/trial` should work with the real device (test dry run)
