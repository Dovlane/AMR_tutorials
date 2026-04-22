# ROS 2 Nodes, Topics, and Services

This note explains three core ROS 2 concepts:

1. Nodes
2. Topics
3. Services

These are the basic building blocks of most ROS 2 systems.

## What Is a Node

A node is a single running ROS 2 program with a specific responsibility.

Examples of responsibilities:

- reading data from a sensor
- controlling a motor
- publishing robot position
- listening for commands
- answering service requests

In practice, a robot application is usually split into multiple nodes instead of one large program.

Why this is useful:

- each node can focus on one task
- nodes can run independently
- nodes can communicate with each other through ROS 2
- systems become easier to debug and extend

In Python with `rclpy`, a node is usually created by inheriting from `Node`.

Example from this repository:

File: [talker.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/talker.py)

```python
class Talker(Node):
    def __init__(self):
        super().__init__('talker')
```

Here:

- `Talker` is the Python class
- `talker` is the ROS 2 node name

Another example:

File: [listener.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/listener.py)

```python
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
```

So `talker` and `listener` are two separate ROS 2 nodes.

## What Is a Topic

A topic is a named communication channel used to stream messages between nodes.

Topics are used when:

- one node produces data
- another node wants to consume that data
- communication should be continuous or event-based

This is called publish/subscribe communication.

### Publisher

A publisher sends messages to a topic.

In this repository, `talker` is a publisher:

```python
self.publisher = self.create_publisher(String, 'chatter', 10)
```

This means:

- message type is `String`
- topic name is `chatter`
- `10` is the queue depth

Then it publishes data:

```python
msg = String()
msg.data = 'Hello ROS 2'
self.publisher.publish(msg)
```

### Subscriber

A subscriber receives messages from a topic.

In this repository, `listener` is a subscriber:

```python
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
```

This means:

- it listens to the topic `chatter`
- it expects messages of type `String`
- it runs `listener_callback` when a message arrives

The callback:

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

### How Topics Work

For topic communication to work correctly:

- publisher and subscriber must use the same topic name
- publisher and subscriber must use the same message type

Flow:

```text
publisher node
  -> publishes message to a topic
  -> ROS 2 middleware transports it
  -> subscriber node receives the message
```

### Important Properties of Topics

- Topics are asynchronous.
- A publisher does not wait for a reply.
- A subscriber reacts whenever data arrives.
- One publisher can send to many subscribers.
- Many publishers can also publish to the same topic if the system is designed that way.

Topics are a good fit for:

- sensor data
- status updates
- telemetry
- periodic robot state
- notifications

## What Is a Service

A service is a request/reply communication mechanism.

It is used when one node wants another node to do something and return a result.

This is different from topics:

- topics are streams of messages
- services are one request followed by one response

### Service Server

A service server offers a service and waits for requests.

In this repository, `add_server` creates a service:

File: [add_server.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/add_server.py)

```python
self.srv = self.create_service(AddInt, 'add_int', self.callback)
```

This means:

- service type is `AddInt`
- service name is `add_int`
- `callback` runs whenever a request is received

The callback:

```python
def callback(self, request, response):
    self.get_logger().info(f"Request: {request.a}")
    response.success = True
    return response
```

### Service Client

A service client sends a request and waits for a reply.

In this repository, `add_client` creates a client:

File: [add_client.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/add_client.py)

```python
self.cli = self.create_client(AddInt, 'add_int')
```

Before sending the request, it waits for the server:

```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('service not available, waiting...')
```

Then it sends the request:

```python
self.req.a = a
return self.cli.call_async(self.req)
```

### Service Definition

Every ROS 2 service has a defined request type and response type.

In this repository, the service definition is:

File: [AddInt.srv](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world_interfaces/srv/AddInt.srv)

```srv
int64 a
---
bool success
```

This means:

- request contains `a`
- response contains `success`

### How Services Work

Flow:

```text
client node
  -> sends request to service
  -> ROS 2 middleware delivers request
  -> server node processes it
  -> server creates response
  -> ROS 2 middleware returns response
  -> client receives the result
```

### Important Properties of Services

- Services are request/reply.
- The client expects a response.
- The server only runs its callback when a request arrives.
- Services are usually used for short operations.

Services are a good fit for:

- asking for a calculation
- triggering a robot action
- resetting a system
- requesting configuration information
- performing a one-time command with a result

## Topics vs Services

Here is the main difference:

### Topics

- communication style: publish/subscribe
- direction: one-way per message
- timing: asynchronous
- best for: continuous data flow

Example:

```text
laser scanner -> publishes range data
navigation node -> subscribes to that data
```

### Services

- communication style: request/reply
- direction: two-way
- timing: client waits for response
- best for: commands and queries

Example:

```text
client -> asks "reset odometry"
server -> replies "done"
```

## How They Relate in a ROS 2 System

A complete ROS 2 application usually combines all of these:

- nodes do the work
- topics move streaming data between nodes
- services handle commands or questions between nodes

Simple example:

```text
camera node -> publishes images on a topic
vision node -> subscribes to images and detects objects
control node -> calls a service to reset or start a behavior
```

## Mapping to This Repository

This repository already contains both communication styles.

### Nodes in `hello_world`

- `talker`
- `listener`
- `add_server`
- `add_client`

These executables are registered in [setup.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/setup.py).

### Topic example

- `talker` publishes on `chatter`
- `listener` subscribes to `chatter`

### Service example

- `add_server` provides the service `add_int`
- `add_client` calls the service `add_int`

## One Important Note About This Example

The names `add_client`, `add_server`, and `AddInt` suggest a real addition service, but the current implementation is only a minimal service example.

Right now:

- request contains only one integer
- response contains only one boolean
- the server logs the received number and returns `True`

So conceptually it teaches how a service works, even though it does not yet compute a sum.

## Short Mental Model

If you want a quick way to remember these concepts:

- node = a ROS 2 program
- topic = a message stream between programs
- service = a question/command with a reply

Or even shorter:

```text
Node = who does the work
Topic = how data is broadcast
Service = how a request gets an answer
```

## Typical Commands

After building and sourcing the workspace:

```bash
colcon build
source install/setup.bash
```

Run the topic example:

```bash
ros2 launch hello_world hello_world.launch.py
```

Run the service example:

Terminal 1:

```bash
ros2 run hello_world add_server
```

Terminal 2:

```bash
ros2 run hello_world add_client 5
```

## Summary

ROS 2 systems are built from small programs called nodes.

- Nodes communicate through topics when data should flow continuously.
- Nodes communicate through services when one side needs a reply.

In this package:

- `talker` and `listener` demonstrate topics
- `add_client` and `add_server` demonstrate services
