# Livox Mid-360 Native Module (C++)

Native C++ driver for the Livox Mid-360 LiDAR. Publishes PointCloud2 and IMU
data directly on LCM, bypassing Python for minimal latency.

## Prerequisites

- [LCM](https://lcm-proj.github.io/) (`pacman -S lcm` or build from source)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) — build and install to `/usr/local`
- CMake >= 3.14

### Installing Livox SDK2

```bash
cd ~/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

## Build

```bash
cd dimos/hardware/sensors/lidar/livox/cpp
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

The binary lands at `build/mid360_native`.

CMake automatically fetches [dimos-lcm](https://github.com/dimensionalOS/dimos-lcm)
for the C++ message headers on first configure.

## Usage

Normally launched by `Mid360CppModule` via the NativeModule framework — you
don't run it directly. The Python wrapper passes LCM topic strings and config
as CLI args:

```python
from dimos.hardware.sensors.lidar.livox.cpp.module import Mid360CppModule
from dimos.core.blueprints import autoconnect

autoconnect(
    Mid360CppModule.blueprint(host_ip="192.168.1.5"),
    SomeConsumer.blueprint(),
).build().loop()
```

### Manual invocation (for debugging)

```bash
./build/mid360_native \
    --pointcloud '/pointcloud#sensor_msgs.PointCloud2' \
    --imu '/imu#sensor_msgs.Imu' \
    --host_ip 192.168.1.5 \
    --lidar_ip 192.168.1.155 \
    --frequency 10
```

Topic strings must include the `#type` suffix — this is the actual LCM channel
name used by dimos subscribers.

## File overview

| File | Description |
|---|---|
| `main.cpp` | Livox SDK2 callbacks, frame accumulation, LCM publishing |
| `dimos_native_module.hpp` | Reusable header for parsing NativeModule CLI args |
| `module.py` | Python NativeModule wrapper (`Mid360CppModule`) |
| `CMakeLists.txt` | Build config, fetches dimos-lcm headers automatically |
