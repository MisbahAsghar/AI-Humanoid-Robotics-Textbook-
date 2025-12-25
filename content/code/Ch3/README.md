# Chapter 3 Code Examples

**Chapter**: ROS 2 Fundamentals
**Purpose**: Hands-on demonstrations of ROS 2 communication patterns (topics, services, actions)

---

## Important Note

**ROS 2 Installation Required**: Unlike previous chapters, these examples require ROS 2 Humble installed on Ubuntu 22.04. See Chapter 3, Section 2.1 for installation instructions.

**Alternative**: Use Docker container with ROS 2 pre-installed (see Section 3.3).

---

## Quick Start

### 1. Install ROS 2 Humble

**Ubuntu 22.04 only** (strongly recommended):

```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# Source ROS 2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Create ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python py_pubsub --dependencies rclpy std_msgs example_interfaces

# Copy example files to package
cp temperature_publisher.py ~/ros2_ws/src/py_pubsub/py_pubsub/
cp temperature_subscriber.py ~/ros2_ws/src/py_pubsub/py_pubsub/
cp add_two_ints_server.py ~/ros2_ws/src/py_pubsub/py_pubsub/
cp add_two_ints_client.py ~/ros2_ws/src/py_pubsub/py_pubsub/
cp fibonacci_action_server.py ~/ros2_ws/src/py_pubsub/py_pubsub/
cp fibonacci_action_client.py ~/ros2_ws/src/py_pubsub/py_pubsub/
```

### 3. Update setup.py

Edit `~/ros2_ws/src/py_pubsub/setup.py` to add entry points (see examples in chapter).

### 4. Build and Run

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select py_pubsub --symlink-install
source install/setup.bash

# Run examples (see sections below)
```

---

## Example Scripts

### `temperature_publisher.py` + `temperature_subscriber.py`
**Demonstrates**: Topic-based publish-subscribe communication

**What it does**:
- **Publisher**: Simulates temperature sensor, publishes Float32 readings to `/temperature` topic at 1 Hz
- **Subscriber**: Monitors temperature, logs warnings/errors if thresholds exceeded (25°C warning, 28°C critical)

**Key Concepts**:
- Creating publishers with `self.create_publisher(msg_type, topic, qos_depth)`
- Creating subscribers with `self.create_subscription(msg_type, topic, callback, qos_depth)`
- Timer callbacks for periodic publishing (`self.create_timer(period, callback)`)
- Message types (`std_msgs.msg.Float32`)
- ROS 2 logging (`self.get_logger().info/warn/error()`)

**Running**:
```bash
# Terminal 1: Publisher
ros2 run py_pubsub temperature_publisher

# Terminal 2: Subscriber
ros2 run py_pubsub temperature_subscriber

# Terminal 3: Inspect topic
ros2 topic list
ros2 topic info /temperature
ros2 topic hz /temperature
ros2 topic echo /temperature
```

**Expected Output** (Subscriber):
```
[INFO] [temperature_subscriber]: Temperature OK: 21.34°C
[WARN] [temperature_subscriber]: WARNING: Temperature 25.67°C
[ERROR] [temperature_subscriber]: CRITICAL: Temperature 28.45°C!
```

---

### `add_two_ints_server.py` + `add_two_ints_client.py`
**Demonstrates**: Service-based request-response communication

**What it does**:
- **Server**: Provides `/add_two_ints` service that sums two integers
- **Client**: Sends request with two integers, receives sum response

**Key Concepts**:
- Creating service servers with `self.create_service(srv_type, service_name, callback)`
- Creating service clients with `self.create_client(srv_type, service_name)`
- Asynchronous service calls with `call_async()` and futures
- Waiting for service availability with `wait_for_service(timeout_sec)`
- Service message types (`.srv` files: `example_interfaces.srv.AddTwoInts`)

**Running**:
```bash
# Terminal 1: Server
ros2 run py_pubsub add_two_ints_server

# Terminal 2: Client
ros2 run py_pubsub add_two_ints_client 5 7

# Or use command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

**Expected Output** (Server):
```
[INFO] [add_two_ints_server]: Add two ints server ready
[INFO] [add_two_ints_server]: Request: 5 + 7 = 12
```

**Expected Output** (Client):
```
[INFO] [add_two_ints_client]: Sending request: 5 + 7
[INFO] [add_two_ints_client]: Result: 5 + 7 = 12
```

---

### `fibonacci_action_server.py` + `fibonacci_action_client.py`
**Demonstrates**: Action-based goal-feedback-result communication

**What it does**:
- **Server**: Computes Fibonacci sequence for N numbers with periodic feedback (0.5s per number)
- **Client**: Sends goal (order=10), monitors feedback, receives final result

**Key Concepts**:
- Creating action servers with `ActionServer(node, action_type, action_name, execute_callback)`
- Creating action clients with `ActionClient(node, action_type, action_name)`
- Goal handling (`goal_handle.succeed()`, `goal_handle.canceled()`)
- Sending feedback with `goal_handle.publish_feedback(feedback_msg)`
- Cancellation support (`goal_handle.is_cancel_requested`)
- Action message types (`.action` files: `action_tutorials_interfaces.action.Fibonacci`)

**Prerequisites**:
```bash
# Install action tutorials interfaces
sudo apt install ros-humble-action-tutorials-interfaces
```

**Running**:
```bash
# Terminal 1: Action server
ros2 run py_pubsub fibonacci_action_server

# Terminal 2: Send goal
ros2 run py_pubsub fibonacci_action_client 10

# Or use command line
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

**Expected Output** (Server):
```
[INFO] [fibonacci_action_server]: Fibonacci action server ready
[INFO] [fibonacci_action_server]: Executing goal: compute 10 Fibonacci numbers
[INFO] [fibonacci_action_server]: Feedback: [0, 1, 1]
[INFO] [fibonacci_action_server]: Feedback: [0, 1, 1, 2]
...
[INFO] [fibonacci_action_server]: Goal succeeded! Final: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

**Expected Output** (Client):
```
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1]
...
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

---

## Troubleshooting

### Common Errors

**1. bash: ros2: command not found**
- **Cause**: ROS 2 not sourced
- **Fix**: Run `source /opt/ros/humble/setup.bash` (add to `~/.bashrc` for persistence)

**2. ModuleNotFoundError: No module named 'rclpy'**
- **Cause**: ROS 2 Python packages not installed or not sourced
- **Fix**: Install `ros-humble-ros-base` and source setup script

**3. Package 'py_pubsub' not found**
- **Cause**: Workspace not built or not sourced
- **Fix**: Run `colcon build` and `source install/setup.bash` from workspace root

**4. Nodes don't see each other / No topics visible**
- **Cause 1**: Different `ROS_DOMAIN_ID` (check with `echo $ROS_DOMAIN_ID`)
- **Cause 2**: Firewall blocking multicast UDP (239.255.0.1)
- **Fix 1**: Ensure same domain ID on all machines
- **Fix 2**: Allow multicast: `sudo ufw allow from 224.0.0.0/4`

**5. E: Unable to locate package ros-humble-desktop**
- **Cause**: ROS 2 repository not added or Ubuntu version mismatch
- **Fix**: Ensure Ubuntu 22.04 (`lsb_release -a`), re-run repository setup

**6. ImportError: cannot import name 'AddTwoInts' from 'example_interfaces.srv'**
- **Cause**: example_interfaces package not installed
- **Fix**: `sudo apt install ros-humble-example-interfaces`

**7. Action server not found**
- **Cause**: action_tutorials_interfaces not installed
- **Fix**: `sudo apt install ros-humble-action-tutorials-interfaces`

**8. colcon: command not found**
- **Cause**: colcon not installed
- **Fix**: `sudo apt install python3-colcon-common-extensions`

---

## Command-Line Debugging Tools

```bash
# List nodes
ros2 node list

# Show node details (topics, services, actions)
ros2 node info /node_name

# List and inspect topics
ros2 topic list
ros2 topic info /topic_name
ros2 topic echo /topic_name    # Print messages in real-time
ros2 topic hz /topic_name      # Measure publish rate

# List and call services
ros2 service list
ros2 service type /service_name
ros2 service call /service_name <type> "<request>"

# List and send action goals
ros2 action list
ros2 action info /action_name
ros2 action send_goal /action_name <type> "<goal>"

# Visualize node graph
rqt_graph  # Opens GUI showing node connections
```

---

## System Requirements

**Minimum**:
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) — REQUIRED
- **CPU**: 2-core x86_64 or ARM64
- **RAM**: 4GB
- **Disk**: 20GB free space

**Recommended**:
- **OS**: Ubuntu 22.04 LTS
- **CPU**: 4-core x86_64
- **RAM**: 8GB
- **Disk**: 30GB free space

**Alternative Platforms** (requires Docker):
- Windows 10/11 with WSL2 + Docker
- macOS with Docker Desktop
- Ubuntu 20.04 (use ROS 2 Galactic instead of Humble)

---

## Related Chapter Content

These code examples support concepts in:
- Section 1.2: Communication Patterns (topics, services, actions)
- Section 2.2: Creating Your First ROS 2 Package
- Section 2.3: Practice Example 1 - Publisher-Subscriber
- Section 2.4: Practice Example 2 - Service
- Section 2.5: Practice Example 3 - Action
- Section 2.6: ROS 2 Command-Line Tools

For theoretical background, see `content/chapters/Ch3.md` sections 1.1-2.6.

---

## Docker Alternative (Non-Ubuntu Platforms)

If you cannot install Ubuntu 22.04 natively, use Docker:

```dockerfile
# Dockerfile
FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-example-interfaces \
    ros-humble-action-tutorials-interfaces

WORKDIR /root/ros2_ws

CMD ["/bin/bash"]
```

**Build and run**:
```bash
docker build -t ros2-humble .
docker run -it --rm --net=host ros2-humble

# Inside container
source /opt/ros/humble/setup.bash
# ... continue with workspace setup
```

---

## Contact / Issues

If code examples don't run as expected:
1. Verify Ubuntu version: `lsb_release -a` (must be 22.04)
2. Verify ROS 2 installation: `ros2 --version` (should show "ros2 humble")
3. Verify sourcing: `echo $AMENT_PREFIX_PATH` (should include /opt/ros/humble)
4. Check firewall: `sudo ufw status` (allow multicast if nodes can't discover)
5. Review troubleshooting section above

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, ROS 2 Humble, Python 3.10
