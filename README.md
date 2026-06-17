
**Workspace Setup:**

Before running any commands, build the workspace and source the setup script:

```bash
colcon build
source install/setup.bash
```

## Usage Instructions:



# goal_pose_cmd

ROS2 (Humble) bridge between **RoboDK** and external clients (Isaac Sim, RViz,
custom scripts). Nodes stream joints / TCP pose out of RoboDK, accept goal
poses to drive the robot, and expose a small pygame UI to command the gripper.

```
                                +-----------------+
                                |                 |
   PoseStamped /goal_pose ----> | goal_pose_cmd_  | --> RoboDK robot moves
                                | node            |
                                +-----------------+

   RoboDK robot pose --------+
                             |  +-----------------+
                             +->| read_tcp_pose_  | --> /tcp_pose (PoseStamped)
   Empty /get_tcp_pose ------>  | node            |
                                +-----------------+

   RoboDK robot joints ------+   +---------------------+
                             |   |                     |
                             +-->| joint_state_        |--> /joint_states
   Float64 /gripper_command ---->| streamer            |    (includes gripper)
                                 +---------------------+

   Float64 /gripper_command <-- gripper_button (pygame UI)
                             <-- gripper_pub      (CLI tool)
```

---

## Build & source

```bash
cd ~/robodk_ros_ws
colcon build --packages-select goal_pose_cmd
source install/setup.bash
```

Requires the RoboDK desktop app running (default API port `20500`) for any
node that talks to RoboDK.

---

## Nodes

### 1. `goal_pose_cmd_node` — pose → RoboDK move

Subscribes to `geometry_msgs/PoseStamped` and tells the first robot in the
RoboDK station to `MoveJ` (default) or `MoveL` there.

| | |
|--|--|
| Executable | `ros2 run goal_pose_cmd goal_pose_cmd_node` |
| Subscribes | `/goal_pose` (`geometry_msgs/PoseStamped`) — units: **meters + quaternion** |
| Publishes | — |
| Parameters | `topic` (`/goal_pose`), `move_type` (`joint`\|`linear`), `speed_mm_s` (`200.0`), `robodk_host` (`localhost`), `robodk_port` (`20500`) |

**Run:**
```bash
ros2 run goal_pose_cmd goal_pose_cmd_node
# linear moves instead of joint moves, slower
ros2 run goal_pose_cmd goal_pose_cmd_node --ros-args -p move_type:=linear -p speed_mm_s:=80.0
```

**Send a goal pose (0.5 m forward, 0.4 m up, gripper pointing down):**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "base"},
  pose: {
    position:    {x: 0.5, y: 0.0, z: 0.4},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  }
}'
```

---

### 2. `read_tcp_pose_node` — RoboDK TCP → ROS

Publishes the current end-effector pose at a fixed rate and on demand.

| | |
|--|--|
| Executable | `ros2 run goal_pose_cmd read_tcp_pose_node` |
| Subscribes | `/get_tcp_pose` (`std_msgs/Empty`) — triggers an immediate read |
| Publishes | `/tcp_pose` (`geometry_msgs/PoseStamped`) — meters + quaternion, `frame_id="base"` |
| Parameters | `publish_rate` (`10.0` Hz), `pub_topic` (`/tcp_pose`), `trigger_topic` (`/get_tcp_pose`), `robodk_host`, `robodk_port` |

**Run:**
```bash
ros2 run goal_pose_cmd read_tcp_pose_node
```

**Watch the stream:**
```bash
ros2 topic echo /tcp_pose
```

**Force an immediate read:**
```bash
ros2 topic pub --once /get_tcp_pose std_msgs/Empty '{}'
```

---

### 3. `joint_state_streamer` — RoboDK arm + gripper → /joint_states

Reads joints from the user-picked RoboDK robot (pop-up at startup) and
publishes them as a `sensor_msgs/JointState`. Also subscribes to
`/gripper_command` and **appends** that value as a `gripper_slider` joint
position in the same `JointState` message — so an Isaac Sim importer that
only listens to `/joint_states` gets both arm and gripper for free.

| | |
|--|--|
| Executable | `ros2 run goal_pose_cmd joint_state_streamer` |
| Subscribes | `/gripper_command` (`std_msgs/Float64`) |
| Publishes | `/joint_states` (`sensor_msgs/JointState`) at `publish_rate` Hz |
| Parameters | `publish_rate` (`50.0`), `topic` (`/joint_states`), `joint_names` (`joint_1,...,joint_6`), `joint_signs` (`1,1,1,1,1,1`), `gripper_joint_name` (`gripper_slider`, set to `""` to disable), `gripper_command_topic` (`/gripper_command`), `gripper_initial_value` (`-25.0`) |

Angles are converted from RoboDK degrees → ROS radians; `joint_signs` lets
you flip individual axes if your simulator's convention differs.

**Run with default KUKA names:**
```bash
ros2 run goal_pose_cmd joint_state_streamer
```

**Run without the gripper joint (pure arm stream):**
```bash
ros2 run goal_pose_cmd joint_state_streamer --ros-args -p gripper_joint_name:=""
```

**Override joint names / signs (e.g. flip joint 3 and 5):**
```bash
ros2 run goal_pose_cmd joint_state_streamer --ros-args \
  -p joint_names:="A1,A2,A3,A4,A5,A6" \
  -p joint_signs:="1,1,-1,1,-1,1"
```

**Verify the stream:**
```bash
ros2 topic hz /joint_states
ros2 topic echo --once /joint_states
```

---

### 4. `ur_joint_state_streamer` — same, for UR robots

UR-flavored variant of node 3: 6 arm joints only, no gripper handling.

| | |
|--|--|
| Executable | `ros2 run goal_pose_cmd ur_joint_state_streamer` |
| Publishes | `/joint_states` (`sensor_msgs/JointState`) at `publish_rate` Hz |
| Parameters | `publish_rate` (`50.0`), `topic` (`/joint_states`), `joint_names` (UR defaults: `shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint`), `joint_signs` (`1,1,1,1,1,1`) |

**Run:**
```bash
ros2 run goal_pose_cmd ur_joint_state_streamer
```

---

### 5. `gripper_button` — pygame UI for the gripper

Three-button window publishing `std_msgs/Float64` on `/gripper_command`.
Pairs with `joint_state_streamer` (which forwards the value into the
`gripper_slider` joint).

| Button | Key | Value |
|--|--|--|
| **OPEN**  | `O`         | `-25.0` |
| **STOP**  | `S`         | `  0.0` |
| **CLOSE** | `C` / space | ` 25.0` |
| quit      | `Q` / Esc   | —       |

| | |
|--|--|
| Executable | `ros2 run goal_pose_cmd gripper_button` |
| Publishes | `/gripper_command` (`std_msgs/Float64`) |
| Parameters | `topic` (`/gripper_command`), `initial_state` (`-25.0`) |

**Run:**
```bash
ros2 run goal_pose_cmd gripper_button
```

**Publish to a different topic:**
```bash
ros2 run goal_pose_cmd gripper_button --ros-args -p topic:=/left_gripper/command
```

---

### 6. `gripper_pub.py` — headless gripper CLI (standalone script)

A no-UI alternative to `gripper_button` for scripting/CI. Publishes the given
value at 10 Hz on **both** `/gripper_command` (Float64) and `/joint_states`
(JointState with only `gripper_slider`). Not registered as a `ros2 run`
entry point — invoke with `python3`.

```bash
# open
python3 /home/qasob/robodk_ros_ws/src/goal_pose_cmd/goal_pose_cmd/gripper_pub.py -25
# stop
python3 /home/qasob/robodk_ros_ws/src/goal_pose_cmd/goal_pose_cmd/gripper_pub.py 0
# close
python3 /home/qasob/robodk_ros_ws/src/goal_pose_cmd/goal_pose_cmd/gripper_pub.py 25
```

Press Ctrl-C to stop.

> Caveat: this publishes a partial `JointState` (gripper only). If you also
> run `joint_state_streamer` you'll get two competing publishers on
> `/joint_states`. For Isaac use either one or the other.

---

### 7. `setup_station.py` — populate the RoboDK station (standalone script)

One-shot helper that loads the KUKA KR16-2, attaches a gripper TCP (z+160 mm),
adds four `LiptonBox_*` pickup frames, and builds a `StackPro_PickPlace`
program. Run once after opening an empty RoboDK station.

```bash
# default (localhost:20500)
python3 /home/qasob/robodk_ros_ws/src/goal_pose_cmd/goal_pose_cmd/setup_station.py
# remote RoboDK, wiping the station first
python3 /home/qasob/robodk_ros_ws/src/goal_pose_cmd/goal_pose_cmd/setup_station.py --host 192.168.1.50 --port 20500 --clear
```

---

## Topic cheat-sheet

| Topic | Type | Direction | Owner |
|--|--|--|--|
| `/goal_pose`       | `geometry_msgs/PoseStamped` | external → ROS  | sub: `goal_pose_cmd_node` |
| `/tcp_pose`        | `geometry_msgs/PoseStamped` | ROS → external  | pub: `read_tcp_pose_node` |
| `/get_tcp_pose`    | `std_msgs/Empty`            | external → ROS  | sub: `read_tcp_pose_node` |
| `/joint_states`    | `sensor_msgs/JointState`    | ROS → external  | pub: `joint_state_streamer` / `ur_joint_state_streamer` |
| `/gripper_command` | `std_msgs/Float64`          | external → ROS  | sub: `joint_state_streamer`, pub: `gripper_button` / `gripper_pub` |
| `/camera_info`     | `sensor_msgs/CameraInfo`    | environment → ROS | pub: environment camera |
| `/depth_h5c`       | `sensor_msgs/Image`         | environment → ROS | pub: environment camera (depth) |
| `/rgb_h5c`         | `sensor_msgs/Image`         | environment → ROS | pub: environment camera (RGB) |

### Environment / sensor topics

These topics are published by the environment (the camera in the cell) — the
ROS nodes in this package consume them, they are not produced here.

```text
$ ros2 topic info /camera_info
Type: sensor_msgs/msg/CameraInfo
Publisher count: 1
Subscription count: 0

$ ros2 topic info /depth_h5c
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0

$ ros2 topic info /rgb_h5c
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0
```

**Inspect them** (use `--no-arr` on the image topics so the multi-MB pixel
buffer isn't dumped to the terminal):
```bash
ros2 topic echo --once /camera_info
ros2 topic echo --once --no-arr /rgb_h5c
ros2 topic echo --once --no-arr /depth_h5c
```

**Example `/camera_info` (`sensor_msgs/CameraInfo`) message:**
```yaml
header:
  stamp:
    sec: 248
    nanosec: 483346292
  frame_id: sim_camera
height: 720
width: 1280
distortion_model: plumb_bob
d: [0.0, 0.0, 0.0, 0.0, 0.0]
k: [3814.06420626281,              0.0,  640.0,
                  0.0, 2545.968820785783,  360.0,
                  0.0,              0.0,    1.0]
r: [1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0]
p: [3814.06420626281,              0.0,  640.0, 0.0,
                  0.0, 2545.968820785783,  360.0, 0.0,
                  0.0,              0.0,    1.0, 0.0]
binning_x: 0
binning_y: 0
roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}
```

**Example `/rgb_h5c` (`sensor_msgs/Image`) message** (shown with `--no-arr`, the
2,764,800-byte `data` array elided):
```yaml
header:
  stamp:
    sec: 252
    nanosec: 166679818
  frame_id: sim_camera
height: 720
width: 1280
encoding: rgb8
is_bigendian: 0
step: 3840                       # width * 3 bytes/pixel
data: '<sequence type: uint8, length: 2764800>'
```

**Example `/depth_h5c` (`sensor_msgs/Image`) message** (shown with `--no-arr`, the
3,686,400-byte `data` array elided):
```yaml
header:
  stamp:
    sec: 254
    nanosec: 916679961
  frame_id: sim_camera
height: 720
width: 1280
encoding: 32FC1                  # 32-bit float depth, meters
is_bigendian: 0
step: 5120                       # width * 4 bytes/pixel
data: '<sequence type: uint8, length: 3686400>'
```

### Gripper values

The streamer publishes the raw value it receives — no URDF-side scaling.
Convention in this workspace:

| Value | Meaning |
|--|--|
| `-25.0` | open  |
| ` 0.0`  | stop / hold |
| `25.0`  | close |

**Drive the gripper from the command line:**
```bash
# open
ros2 topic pub --once /gripper_command std_msgs/Float64 '{data: -25.0}'
# stop
ros2 topic pub --once /gripper_command std_msgs/Float64 '{data:   0.0}'
# close
ros2 topic pub --once /gripper_command std_msgs/Float64 '{data:  25.0}'
```

---

## Services

### `/move_conveyor` — start / stop the conveyor

A `std_srvs/srv/SetBool` service that toggles the conveyor: `data: true` runs
it, `data: false` stops it. The response uses the standard `SetBool` reply
(`success` + an optional `message`).

| | |
|--|--|
| Service | `/move_conveyor` |
| Type | `std_srvs/srv/SetBool` |
| Request | `bool data` — `true` = run, `false` = stop |
| Response | `bool success`, `string message` |

**Discover it:**
```bash
ros2 service list
# /move_conveyor
ros2 service type /move_conveyor
# std_srvs/srv/SetBool
```

**Call it:**
```bash
# stop the conveyor
ros2 service call /move_conveyor std_srvs/srv/SetBool "{data: false}"
# start the conveyor
ros2 service call /move_conveyor std_srvs/srv/SetBool "{data: true}"
```

**Example response:**
```text
requester: making request: std_srvs.srv.SetBool_Request(data=False)

response:
std_srvs.srv.SetBool_Response(success=True, message='')
```

---

## Typical session (KUKA + Isaac Sim)

In four terminals, after `source install/setup.bash`:

```bash
# 1. start RoboDK GUI, then populate the demo station once
python3 src/goal_pose_cmd/goal_pose_cmd/setup_station.py

# 2. stream joints (arm + gripper) from RoboDK to Isaac
ros2 run goal_pose_cmd joint_state_streamer

# 3. stream live TCP pose out of RoboDK (optional, for monitoring)
ros2 run goal_pose_cmd read_tcp_pose_node

# 4. control the gripper from the pygame UI
ros2 run goal_pose_cmd gripper_button
```

Then either drive RoboDK manually / via its program, or send `/goal_pose`
messages from your planner and let `goal_pose_cmd_node` move the robot.



---
---
---






















### Open Scene in IsaacSim:
```bash
src/usd_scenes/stackpro_democell.usd
```

- **Send goal to end-effector pose:**
	```bash
	ros2 run goal_pose_cmd goal_pose_cmd_node
	```

- **Read end-effector pose:**

NOTE: robot should already be imported in RoboDK and its frame should be set as active frame

	```bash
	ros2 run goal_pose_cmd read_tcp_pose_node
	ros2 topic echo /tcp_pose
	```

- **Bridge Isaac Sim with RoboDK:**
	```bash
	ros2 run goal_pose_cmd joint_state_streamer
	```
	*Note: Import the robot in Isaac Sim and create a joint state subscriber using Action Graphs with the topic name `/joint_states`.*

- **Sample URDF robot for Isaac Sim:**
	```bash
	~/robodk_ros_ws/src/kuka_robot_descriptions/kuka_quantec_support/urdf/kr240_r2900_2.urdf
	```