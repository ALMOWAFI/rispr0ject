# Memory Game - Franka Panda + ROS1

Memory game for a Franka Emika Panda arm. Vision detects the colored blocks and player actions, the game node generates the memory sequence and checks the response, and the motion node shows the sequence on the robot.

Team: Ali, Sinan, Izat, Boburjon

## Current System

The current real-robot flow is:

```text
RGB + depth topics -> vision_node -> /detected_blocks, /player_selection
                                -> game_node -> /target_sequence, /game_state, /score
                                           -> motion_moveit_node
                                           -> /motion_status
```

Important current behavior:

- `game_node` generates a full round sequence and publishes it as `memory_game/BlockSequence` on `/target_sequence`
- `motion_moveit_node` executes that sequence block-by-block
- the current motion implementation is non-Cartesian hover -> point -> hover using standard MoveIt planning
- `motion_status` includes sequence-aware feedback so `game_node` can verify what was actually shown

## Main Nodes

- `vision_node`
  - detects blocks from RGB + depth
  - publishes `/detected_blocks`
  - publishes `/player_selection`

- `game_node`
  - owns round logic, score, and sequence generation
  - subscribes to `/detected_blocks`, `/player_selection`, `/motion_status`
  - publishes `/target_sequence`, `/game_state`, `/score`

- `motion_moveit_node`
  - current real-robot motion backend
  - subscribes to `/target_sequence`
  - publishes `/motion_status`
  - uses MoveIt planned motion for hover -> point -> hover

- `motion_hw_node`
  - fallback bridge for setups that expose a pose command topic instead of MoveIt

- `motion_node`
  - demo/RViz-only marker motion
  - not the real robot path

## Main Topics

- `/detected_blocks` - `memory_game/BlockArray`
- `/player_selection` - `memory_game/PlayerSelection`
- `/target_sequence` - `memory_game/BlockSequence`
- `/motion_status` - `std_msgs/String`
- `/game_state` - `std_msgs/String`
- `/score` - `std_msgs/Int32`

## Motion Status Contract

The current `game_node` expects these motion states:

- `MOVING_TO_TARGET:<sequence_id>:<block_id>`
- `POINTING:<sequence_id>:<block_id>`
- `AT_TARGET:<sequence_id>:<block_id>`
- `RETURNING_HOME:<sequence_id>:-1`
- `MOVE_FAILED:<sequence_id>:<block_id>`
- `IDLE:<sequence_id>:-1`

## Build

```bash
cd ~/catkin_ws
catkin_make --pkg memory_game
source devel/setup.bash
```

Notes:

- `motion_moveit_node` is only built when MoveIt is available in that workspace
- the package name is `memory_game` even though the repo directory may be named `risproject`

## Real Robot Run

### 1. Start the robot stack / MoveIt

Example:

```bash
roslaunch cu_panda_bringup hardware.launch
```

### 2. Run the ROS nodes manually

Motion:

```bash
rosparam load $(rospack find memory_game)/config/game_params.yaml
rosrun memory_game motion_moveit_node _planning_group:=arm
```

Vision:

```bash
rosparam load $(rospack find memory_game)/config/game_params.yaml
rosrun memory_game vision_node
```

Game:

```bash
rosparam load $(rospack find memory_game)/config/game_params.yaml
rosrun memory_game game_node
```

## UI / Operator Interface

The repo also contains a lightweight browser UI in `ui/`.

It is launched with:

```bash
cd ~/catkin_ws/src/risproject
source ~/catkin_ws/devel/setup.bash
python3 ui/server.py
```

Default URL:

```text
http://127.0.0.1:8000
```

What the UI does:

- shows live game, motion, and perception state
- can start the managed session processes from the browser
- uses `ui/server.py` as the backend bridge

Important:

- the UI is linked to process startup, not to a single `roslaunch` file
- the Panda bringup / MoveIt stack still needs to be running first

## Launch Files

- `launch/full_system.launch`
  - convenience launch
  - can start `motion_moveit_node` when `use_real_robot:=true`
  - still not the main source of truth for the lab workflow

- `launch/test_rviz.launch`
  - demo/debug path

- `launch/vision_only.launch`
  - vision helper

## Current Project Status

Current practical state:

- block detection works in the current 3-block lab mode
- game logic is integrated with real block detections
- sequence-based game -> motion integration is working
- the lab MoveIt group is `arm`
- the current motion node avoids Cartesian dip because the robot was smoother with standard MoveIt planned motions

Remaining limitations:

- red is still unreliable in the real scene
- player selection can still be improved
- motion still needs final real-robot tuning for speed, heights, and reliability under repeated runs

## Repo Layout

- `src/vision_node.cpp` - RGB/depth block detection and player selection
- `src/game_node.cpp` - round logic, sequence generation, scoring, motion requests
- `src/motion_node.cpp` - demo marker motion only
- `newmotion/motion_moveit_node.cpp` - current real-robot MoveIt backend
- `newmotion/motion_hw_node.cpp` - pose-topic fallback backend
- `config/game_params.yaml` - runtime parameters
- `msg/BlockSequence.msg` - full sequence message sent from game to motion
- `ui/` - browser UI and ROS bridge

## Troubleshooting

- `motion_moveit_node` says planning group not found:
  - run with `_planning_group:=arm`

- `motion_moveit_node` missing after build:
  - MoveIt is not installed in that workspace

- game waits for motion:
  - check that `motion_moveit_node` is running and subscribed to `/target_sequence`

- game waits for blocks:
  - vision is not publishing fresh valid detections yet

- message type unknown:
  - rebuild the workspace and source `devel/setup.bash` again

- UI starts but live state is empty:
  - source the ROS and catkin environment before launching `ui/server.py`
