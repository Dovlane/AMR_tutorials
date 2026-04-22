# `hello_world` Example Explained

This package shows two different ROS 2 communication patterns:

1. Topic communication: `talker` publishes data and `listener` subscribes to it.
2. Service communication: `add_client` sends a request and `add_server` replies.

## Where the executables come from

The executables are registered in [setup.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/setup.py):

- `talker = hello_world.talker:main`
- `listener = hello_world.listener:main`
- `add_server = hello_world.add_server:main`
- `add_client = hello_world.add_client:main`

That means ROS 2 can run them as console executables after the package is built.

Important: the files under `build/hello_world/build/lib/hello_world/` are generated build outputs.  
The real source files you should study and edit are in:

- [hello_world/talker.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/talker.py)
- [hello_world/listener.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/listener.py)
- [hello_world/add_server.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/add_server.py)
- [hello_world/add_client.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/add_client.py)

## 1. `talker` and `listener`

### `talker`

File: [hello_world/talker.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/talker.py)

What it does:

- Creates a node named `talker`
- Creates a publisher on the topic `chatter`
- Uses message type `std_msgs/msg/String`
- Starts a timer that fires every `1.0` second
- Every second, publishes `"Hello ROS 2"`

Relevant code flow:

```python
self.publisher = self.create_publisher(String, 'chatter', 10)
self.timer = self.create_timer(1.0, self.on_timer)
```

When the timer triggers:

```python
msg = String()
msg.data = 'Hello ROS 2'
self.publisher.publish(msg)
```

So `talker` continuously sends messages to the topic `chatter`.

### `listener`

File: [hello_world/listener.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/listener.py)

What it does:

- Creates a node named `listener`
- Subscribes to the topic `chatter`
- Expects messages of type `std_msgs/msg/String`
- Runs `listener_callback` every time a message arrives

Relevant code flow:

```python
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
```

When a message arrives:

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

### How they interact

The interaction is indirect. `talker` and `listener` do not know about each other by node name.

They are connected because:

- both use the same topic name: `chatter`
- both use the same message type: `String`

Flow:

```text
talker node
  -> publishes String message on topic "chatter"
  -> ROS 2 middleware delivers it
  -> listener node receives it through its subscription callback
```

This is asynchronous communication:

- `talker` just publishes and continues
- `listener` reacts whenever data arrives
- one publisher can talk to many subscribers

## 2. `add_client` and `add_server`

These two executables use a ROS 2 service, which is request/reply communication.

## The service definition

The service type is defined in [AddInt.srv](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world_interfaces/srv/AddInt.srv):

```srv
int64 a
---
bool success
```

This means:

- request contains one field: `a`
- response contains one field: `success`

Despite the names `add_client` and `add_server`, this service does not currently add numbers.  
It only sends one integer to the server, and the server returns `success = True`.

### `add_server`

File: [hello_world/add_server.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/add_server.py)

What it does:

- Creates a node named `add_server`
- Advertises a service named `add_int`
- Uses service type `AddInt`
- Waits for requests from clients

Relevant code:

```python
self.srv = self.create_service(AddInt, 'add_int', self.callback)
```

When a request arrives:

```python
def callback(self, request, response):
    self.get_logger().info(f"Request: {request.a}")
    response.success = True
    return response
```

So the server:

- receives the integer `request.a`
- prints it in the log
- sets `response.success = True`
- sends the response back to the client

### `add_client`

File: [hello_world/add_client.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/hello_world/add_client.py)

What it does:

- Creates a node named `add_client`
- Creates a client for the service `add_int`
- Waits until the service becomes available
- Takes an integer from the command line
- Sends that integer as a request
- Waits for the reply

Relevant code:

```python
self.cli = self.create_client(AddInt, 'add_int')
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('service not available, waiting...')
```

Sending the request:

```python
self.req.a = a
return self.cli.call_async(self.req)
```

In `main()`:

```python
future = node.send_request(int(sys.argv[1]))
rclpy.spin_until_future_complete(node, future)
print(f"Success = {future.result().success}")
```

### How they interact

This interaction is direct and synchronous from the client's point of view.

Flow:

```text
add_client
  -> waits for service "add_int"
  -> sends request {a = some_integer}
  -> ROS 2 middleware delivers request
  -> add_server callback runs
  -> server creates response {success = True}
  -> ROS 2 middleware sends response back
  -> client receives response and prints it
```

Unlike topics:

- the client expects one reply for each request
- the server only does work when a request arrives
- this pattern is good for commands, queries, or operations with a clear result

## Launch file behavior

File: [launch/hello_world.launch.py](/home/vladimir/workspace/AMR_tutorials/Kodovi/hello_world/launch/hello_world.launch.py)

This launch file starts:

- `talker`
- `listener`

So if you run the launch file, you will see the publish/subscribe example working automatically.

It does **not** start:

- `add_server`
- `add_client`

Those service executables must be started separately.

## Interaction summary

### Publish/subscribe pair

```text
talker publishes to topic "chatter"
listener subscribes to topic "chatter"
```

- communication style: one-way stream of messages
- coupling: loose
- timing: asynchronous

### Service pair

```text
add_client calls service "add_int"
add_server serves "add_int"
```

- communication style: request/reply
- coupling: tighter than topics
- timing: client waits for response

## Typical commands

Build:

```bash
colcon build
source install/setup.bash
```

Run the topic example:

```bash
ros2 launch hello_world hello_world.launch.py
```

Run the service example in two terminals:

Terminal 1:

```bash
ros2 run hello_world add_server
```

Terminal 2:

```bash
ros2 run hello_world add_client 5
```

Expected result:

- server logs `Request: 5`
- client prints `Success = True`

## One subtle point

If your goal is a real "add two integers" example, the current service definition and code are incomplete.

Right now:

- request has only one number: `a`
- response has only one boolean: `success`

For a real add service, you would usually define something like:

```srv
int64 a
int64 b
---
int64 sum
```

Then the server would compute `a + b`, and the client would print the sum.
