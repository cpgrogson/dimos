
# Dimos Transports

Transports enable communication between [modules](modules.md) across process boundaries and networks. When modules run in different processes or on different machines, they need a transport layer to exchange messages.

## Unconventional Transports

Transports (theoretically) aren't limited to traditional pub-sub services. By implementing just broadcast and subscribe functions on `Transport` at [`core/stream.py`](/dimos/core/stream.py#L83) subclass you can do anything.

Your arguments to Transport can be an IP address and port, a shared memory segment name, a file path, or a Redis channel.

For example rebinding an existing go2 blueprint to use (imagined) TCP transport for lidar (each module that requires lidar data would connect via TCP to this ip.


```python skip
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import nav

# use TCP for lidar data (each module individually would establish a tcp connection)
ros = nav.transports(
   {("lidar", PointCloud2): TCPTransport(ip="10.10.10.1",port=1414)}
)
```

subscribe() of your TCP transport just needs to return a standard PointCloud2 object, we don't care how you transport, encode or construct it, though all our types provide `lcm_encode` and `lcm_decode` functions for (faster then pickle and language agnostic) binary encoding. for more info on this, check [lcm](/docs/concepts/lcm.md)

## PubSub Transports

For now at dimos, we've been using exclusively PubSub protocols for our transports `PubSub` abstract class is what those protocol implementations conform to.

```python session=pubsub_demo ansi=false
from dimos.protocol.pubsub.spec import PubSub

# The interface every transport must implement:
import inspect
print(inspect.getsource(PubSub.publish))
print(inspect.getsource(PubSub.subscribe))
```

<!--Result:-->
```
    @abstractmethod
    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Publish a message to a topic."""
        ...

    @abstractmethod
    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe to a topic with a callback. returns unsubscribe function"""
        ...
```

So new protocols are very easy to implement.

Key points:
- `publish(topic, message)` - Send a message to all subscribers on a topic
- `subscribe(topic, callback)` - Register a callback, returns an unsubscribe function

We don't tell you what topic type actually is, it can be a complex configuration object, and we also don't tell you what a message is. it can be bytes, json etc. If you accept `DimosMsg` [`msgs/protocol.py`](/dimos/msgs/protocol.py#L19) you'll be able to transport any dimos msg, but we don't prevent you from implementing bytes only transports, video streams etc

## Implementing a Simple Transport

The simplest toy transport is `Memory`, which works within a single process, start from there,

```python session=memory_demo ansi=false
from dimos.protocol.pubsub.memory import Memory

# Create a memory transport
bus = Memory()

# Track received messages
received = []

# Subscribe to a topic
unsubscribe = bus.subscribe("sensor/data", lambda msg, topic: received.append(msg))

# Publish messages
bus.publish("sensor/data", {"temperature": 22.5})
bus.publish("sensor/data", {"temperature": 23.0})

print(f"Received {len(received)} messages:")
for msg in received:
    print(f"  {msg}")

# Unsubscribe when done
unsubscribe()
```

<!--Result:-->
```
Received 2 messages:
  {'temperature': 22.5}
  {'temperature': 23.0}
```

The full implementation is minimal. See [`memory.py`](/dimos/protocol/pubsub/memory.py) for the complete source.

## Encode/Decode Mixins

Transports often need to serialize messages before sending and deserialize after receiving. The `PubSubEncoderMixin` at [`pubsub/spec.py`](/dimos/protocol/pubsub/spec.py#L95) provides a clean way to add encoding/decoding to any pubsub implementation.

### Available Mixins

| Mixin                          | Encoding        | Use Case                              |
|--------------------------------|-----------------|---------------------------------------|
| `PickleEncoderMixin`           | Python pickle   | Any Python object, same-language only |
| `LCMEncoderMixin`              | LCM binary      | Cross-language (C/C++/Python/Java/Go) |
| `JpegEncoderMixin`             | JPEG compressed | Image data, reduces bandwidth         |

The `LCMEncoderMixin` is especially powerful because LCM messages encode to compact binary that works across languages. This means you can use LCM message definitions with *any* transport - not just LCM's UDP multicast. See [lcm](/docs/concepts/lcm.md) for details on LCM message types.

### Creating a Custom Mixin

Subclass `PubSubEncoderMixin` and implement `encode()` and `decode()`:

```python session=jsonencoder
from dimos.protocol.pubsub.spec import PubSubEncoderMixin
import json

class JsonEncoderMixin(PubSubEncoderMixin[str, dict, bytes]):
    def encode(self, msg: dict, topic: str) -> bytes:
        return json.dumps(msg).encode('utf-8')

    def decode(self, msg: bytes, topic: str) -> dict:
        return json.loads(msg.decode('utf-8'))
```

Then combine with a pubsub implementation using multiple inheritance:

```python session=jsonencoder
from dimos.protocol.pubsub import Memory

class MyJsonPubSub(JsonEncoderMixin, Memory):
    pass
```

The mixin automatically wraps `publish()` and `subscribe()` to handle encoding/decoding transparently. Your new transport implementation stays the same - just swap the mixin:

```python session=jsonencoder
from dimos.protocol.pubsub.spec import PickleEncoderMixin

# Switch to pickle - just change the mixin
class MyPicklePubSub(PickleEncoderMixin, Memory):
    pass
```

Same transport, different serialization - no code changes needed in the transport itself.

## Passing tests

### Spec

check [`pubsub/test_spec.py`](/dimos/protocol/pubsub/test_spec.py) for grid tests for your new protocol, make sure to pass those spec tests

### Benchmark

We also have fancy benchmark tests that will tell you your max bandwidth, latency, message throughput etc.

`python -m pytest -svm tool -k "not bytes" dimos/protocol/pubsub/benchmark/test_benchmark.py`

![Benchmark results](assets/pubsub_benchmark.png)

## Available Transports

Dimos includes several transport implementations:

| Transport      | Use Case                               | Process Boundary | Network |
|----------------|----------------------------------------|------------------|---------|
| `Memory`       | Testing, single process                | No               | No      |
| `SharedMemory` | Multi-process on same machine          | Yes              | No      |
| `LCM`          | Network communication (UDP multicast)  | Yes              | Yes     |
| `Redis`        | Network communication via Redis server | Yes              | Yes     |
| `ROS`          | ROS 2 topic communication              | Yes              | Yes     |
| `DDS`          | Cyclone DDS without ROS (WIP)          | Yes              | Yes     |

### SharedMemory Transport

For inter-process communication on the same machine, `SharedMemory` provides high-performance message passing:

```python session=shm_demo ansi=false
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory

shm = PickleSharedMemory(prefer="cpu")
shm.start()

received = []
shm.subscribe("test/topic", lambda msg, topic: received.append(msg))
shm.publish("test/topic", {"data": [1, 2, 3]})

import time
time.sleep(0.1)  # Allow message to propagate

print(f"Received: {received}")
shm.stop()
```

<!--Result:-->
```
Received: [{'data': [1, 2, 3]}]
```

### LCM Transport

For local network communication, LCM uses UDP multicast and supports typed messages:

```python session=lcm_demo ansi=false
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.msgs.geometry_msgs import Vector3

lcm = LCM(autoconf=True)
lcm.start()

received = []
topic = Topic(topic="/robot/velocity", lcm_type=Vector3)

lcm.subscribe(topic, lambda msg, t: received.append(msg))
lcm.publish(topic, Vector3(1.0, 0.0, 0.5))

import time
time.sleep(0.1)

print(f"Received velocity: x={received[0].x}, y={received[0].y}, z={received[0].z}")
lcm.stop()
```

<!--Result:-->
```
Received velocity: x=1.0, y=0.0, z=0.5
```

### Inspecting LCM traffic (CLI)

- `dimos lcmspy` shows topic frequency/bandwidth stats.
- `dimos topic echo /topic` listens on typed channels like `/topic#pkg.Msg` and decodes automatically.
- `dimos topic echo /topic TypeName` is the explicit legacy form.

## Using Transports with Modules

Every module stream can use a different transport. Set `.transport` on the stream before starting the module:

```python ansi=false
import time

from dimos.core import In, Module, start
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.msgs.sensor_msgs import Image


# Define a simple listener module
class ImageListener(Module):
    image: In[Image]

    def start(self):
        super().start()
        self.image.subscribe(lambda img: print(f"Received: {img.shape}"))


if __name__ == "__main__":
    # Start cluster and deploy modules to separate processes
    dimos = start(2)

    camera = dimos.deploy(CameraModule, frequency=2.0)
    listener = dimos.deploy(ImageListener)

    # Connect via shared memory transport (pSHMTransport uses pickle encoding)
    camera.color_image.transport = pSHMTransport("/camera/rgb")
    listener.image.transport = pSHMTransport("/camera/rgb")

    # Start both modules
    camera.start()
    listener.start()

    time.sleep(2)

    dimos.stop()
```

<!--Result:-->
```
Initialized dimos local cluster with 2 workers, memory limit: auto
2026-01-24T07:44:39.770667Z [info     ] Deploying module.                                            [dimos/core/__init__.py] module=CameraModule
2026-01-24T07:44:39.805460Z [info     ] Deployed module.                                             [dimos/core/__init__.py] module=CameraModule worker_id=0
2026-01-24T07:44:39.819562Z [info     ] Deploying module.                                            [dimos/core/__init__.py] module=ImageListener
2026-01-24T07:44:39.849461Z [info     ] Deployed module.                                             [dimos/core/__init__.py] module=ImageListener worker_id=1
Received: (480, 640, 3)
Received: (480, 640, 3)
Received: (480, 640, 3)
```

This is useful when you need to:
- Share data between processes on the same machine (SharedMemory)
- Communicate across the network (LCM, Redis)
- Mix transports for different streams (e.g., low-latency for control, high-bandwidth for video)

See [Modules](modules.md) for more on module architecture.
