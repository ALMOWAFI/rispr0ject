# Memory Game Robot Project Documentation

Team: Ali, Sinan, Izat, Boburjon

## 1. Project Overview

Our project is a ROS1 memory game using a Franka Emika Panda robot arm and a
RealSense RGB-D camera. The idea is simple: the robot shows a sequence by
pointing at colored blocks, then the player repeats the sequence. The system
checks the player's answer and updates the score.

The project did not start as the system that exists now. At the beginning, we
only had a clean ROS package, simple messages, mock block positions, and a
basic game loop. Over time, we added real camera detection, MoveIt robot
motion, player selection, UI controls, sound feedback, and many lab-specific
fixes. A lot of important work was overwritten or reverted during testing, so
the commit history is important for understanding how the project actually
developed.

The final project is not just three files. It is a full ROS package with:

- C++ ROS nodes for vision, game logic, and robot motion
- custom ROS message types
- RealSense RGB/depth processing
- MoveIt control for the Panda robot
- Python player-selection logic
- launch files and tuning parameters
- a browser UI for running and monitoring the game
- documentation and runbooks

## 2. Current System

The current system works like this:

```text
RealSense RGB + aligned depth
        |
        v
vision_node
        |
        +--> /detected_blocks
        |
player_selection.py
        |
        +--> /player_selection

/detected_blocks + /player_selection + /motion_status
        |
        v
game_node
        |
        +--> /target_sequence
        +--> /game_state
        +--> /score

/target_sequence
        |
        v
motion_moveit_node
        |
        +--> /motion_status
```

The current real robot flow uses `/target_sequence`, not the older
`/target_block` design. This was one of the biggest architecture changes in the
project. Earlier versions sent one block target at a time. Later, we changed
the game node to send a full sequence with a sequence ID, so the motion node
could execute the whole robot demonstration and report exactly what happened.

The main current nodes are:

- `src/vision_node.cpp`
  - detects colored blocks from RGB and aligned depth images
  - publishes `/detected_blocks`
  - publishes debug mask and overlay images for tuning

- `scripts/player_selection.py`
  - watches block slots
  - detects a player selection when a block disappears from its original slot
  - publishes `/player_selection`

- `src/game_node.cpp`
  - owns the game state
  - creates the memory sequence
  - waits for valid block detections
  - sends `/target_sequence`
  - checks `/motion_status`
  - checks player input and updates score

- `newmotion/motion_moveit_node.cpp`
  - controls the real Panda robot through MoveIt
  - subscribes to `/target_sequence`
  - moves hover -> point -> hover for every block
  - publishes sequence-aware `/motion_status`

- `src/motion_node.cpp`
  - demo/RViz motion node
  - useful for testing, but not the real robot path

## 3. How We Started

The first commit was on February 20, 2026:

```text
453bd35 - Initial commit: Memory Game ROS1 package - Complete ROS1 package structure with vision, game, and motion nodes
```

This commit created the full starting structure of the project. It included:

- `CMakeLists.txt`
- `package.xml`
- `README.md`
- `ROS1_GUIDE.md`
- `TASK_BREAKDOWN.md`
- `COLLABORATION.md`
- `docs/architecture.md`
- `config/game_params.yaml`
- `launch/` files
- `msg/Block.msg`
- `msg/BlockArray.msg`
- `msg/PlayerSelection.msg`
- `src/vision_node.cpp`
- `src/game_node.cpp`
- `src/motion_node.cpp`
- `scripts/test_player_selection.py`

At this stage the project was mostly a skeleton, but it was already organized
around the final idea:

1. Vision finds blocks and player selections.
2. Game logic creates a memory sequence.
3. Motion points the robot at the selected blocks.

The first vision node did not use the real camera yet. It published four mock
blocks in fixed positions, which let us test the ROS messages, RViz markers,
launch files, and communication between nodes before real hardware was ready.

The first game node already had a basic memory game state machine:

- `IDLE`
- `SHOWING_SEQUENCE`
- `WAITING_PLAYER`
- `CHECKING_INPUT`
- `GAME_OVER`

The first motion node was also mostly for simulation. It let us test the idea
of moving toward a target block without needing the real Panda setup.

This was a good first step because it gave the group something runnable before
we started fighting with real sensors and robot motion.

## 4. Early Simulation And ROS Integration

After the initial commit, the first phase was about making the package build
and behave correctly in RViz.

Important commits in this phase:

- `1bf3858` added a more complete motion state machine with smooth
  interpolation and return-home behavior.
- `268bdd5` improved the vision node and added tuning notes.
- `048c929` replaced the old game node and added a Panda URDF folder.
- `09291c2` improved `motion_node` with 3D distance detection.
- `fae23fe`, `e37d119`, and `09b0dee` adjusted RViz, launch files, and
  parameters after testing errors.
- `3464666` cleaned up launch files and removed the old simulation launch.
- `4b8a207`, `4b1a547`, and `1aad75d` improved manual player-selection
  testing.

During this phase, the system was still not a real robot demo. The goal was to
make sure all parts could talk to each other. We tested topics, launch files,
message types, and RViz visualization. This gave us a foundation before moving
to the lab robot.

## 5. Moving Toward Real Robot Control

Around March 13, the project started moving from a simple simulated motion node
to real robot motion.

Important commits:

- `714b0ac` added a motion interface probe and a hardware motion stub.
- `bc94f58` moved motion helper files into `newmotion/`.
- `05ea1ec` made the game wait for a motion subscriber before publishing.
- `57a6bfc` changed vision defaults to the `/realsense/*` camera topics.
- `3c5ba01` added a MoveIt-backed executor for robot motion.
- `c3eb7fb` added ROI/workspace filters to reduce false positives.

This was when we learned that the real system needed more than the original
three-node idea. The package now had different motion paths:

- `src/motion_node.cpp` for demo/RViz testing
- `newmotion/motion_hw_node.cpp` for a simple hardware pose command fallback
- `newmotion/motion_moveit_node.cpp` for the real Panda MoveIt setup

The game also had to become more careful. It could not just publish a target
and assume the robot was ready. It needed to wait for a subscriber and later
check motion status messages.

## 6. Vision Problems In The Lab

Vision was the hardest part of the project. In simulation, block positions were
clean and predictable. In the lab, the camera saw lighting changes, shadows,
skin colors, background objects, the robot base, and sometimes wrong depth
values.

Several commits show how much work went into this:

- `8b86e0c` excluded skin-colored pixels from block detection.
- `3c1ce39` added a `disable_red` mode.
- `67d690b` hardened lab vision and motion behavior.
- `422823d` hardened coordination between game, vision, and motion.
- `db2782b` synchronized RGB and depth.
- `432472a` added buffered depth matching.
- `309bfdd` added hand-mask debugging and tunable skin HSV.
- `ea3e0e7` allowed the game to start with only three detected blocks.
- `c07788b` improved player selection and updated project status docs.

The biggest recurring issue was red. Red was unreliable because skin tones,
brown objects, and some background areas could overlap with red HSV ranges. We
did not completely remove red from the project, but the current lab setup
disables red by default and uses three stable colors instead.

This is why the current `game_params.yaml` has:

```yaml
disable_red: true
min_detected_blocks_required: 3
```

This was not part of the original plan. It was a practical change based on real
testing.

## 7. RealSense And Depth Debugging

Another major problem was RGB/depth synchronization. Sometimes the node would
not publish useful debug images because it was waiting for depth or camera info.
That made tuning very difficult.

Important commits:

- `b1e14ff` fixed depth/RGB sync blocking all debug image output.
- `ae163a8` fixed zero timestamps causing sync failure.
- `c1f3681` allowed depth processing even with timestamp drift.
- `cb9eb24` started RealSense with `align_depth=true`.
- `df0309d` added `scripts/start_camera.sh`.
- `92f28f9` published debug images even when camera info was missing.
- `4050ca8` added `scripts/run_vision.sh`.

These changes are why the current vision node is designed to be easier to
debug. It can publish overlays such as `NO CAMERA INFO` or `NO DEPTH SYNC`
instead of silently failing. It also uses a depth buffer and chooses the
closest depth frame instead of depending on one perfect timestamp match.

## 8. Player Selection

Player selection changed a lot during the project. Early versions tried to
handle selection directly through the vision node or through separate
experiments. Later, the project moved to a simpler and more reliable idea:
watch the initial block slots and treat a missing block as the player's
selection.

Important commits:

- `47a5e61` added isolated `try_clean` vision experiments.
- `e696a3a` fixed the Python ROS node integration.
- `036fddc` fixed slot selection logic.
- `fd82c33` wired slot selection into launch files.
- `d02cb90` simplified the always-on player-selection flow.
- `a6ea165` cleaned up `player_selection.py`.
- `9df9c57` added auto-reset when all block slots disappear.

The current `player_selection.py` works like this:

1. Wait until enough blocks are detected.
2. Let the detections settle.
3. Save the starting position of each block as a slot.
4. If a block is missing from its slot for long enough, publish a
   `/player_selection`.
5. Reset if all blocks disappear for a while.

This is not a perfect human gesture detector, but it is more practical for our
setup.

## 9. Game Logic Evolution

The game node started with a simple idea: generate a random sequence, publish
one target block at a time, wait for player input, then check if the answer was
correct.

Over time, the game node became more responsible because it had to coordinate
with real vision and motion.

Important game changes:

- The game learned to wait for detected blocks instead of using unsafe default
  positions.
- Red could be disabled.
- The game could start with three blocks.
- The game waited for motion readiness before dispatch.
- The game started checking motion status.
- Motion retries and timeout handling were added.
- Target workspace validation was added so impossible positions would not be
  sent to the robot.

The biggest change happened in commit `a0b6e66`:

```text
2026-04-30 - Integrate sequence-based game and motion flow
```

This commit added:

- `msg/BlockSequence.msg`
- `/target_sequence`
- sequence IDs
- sequence-aware motion status

Before this, the game sent one `/target_block` at a time. After this change,
the game sends a whole sequence to the motion node. This is more reliable
because the robot can execute the sequence as one job and the game can verify
which sequence the motion status belongs to.

The current motion status format includes:

```text
MOVING_TO_TARGET:<sequence_id>:<block_id>
POINTING:<sequence_id>:<block_id>
AT_TARGET:<sequence_id>:<block_id>
RETURNING_HOME:<sequence_id>:-1
MOVE_FAILED:<sequence_id>:<block_id>
IDLE:<sequence_id>:-1
```

This prevents old motion messages from being confused with the current round.

## 10. Robot Motion

Robot motion also changed many times. The project first had a demo motion node.
Then we added a hardware stub. Then we added a MoveIt node for the real Panda.
After that, we spent a lot of time tuning how the robot should point.

Important commits:

- `3c5ba01` added the first MoveIt-backed executor.
- `59e69c4` set the MoveIt planning group to `arm`.
- `ca85340` smoothed Cartesian dip execution.
- `3416b6e` replaced Cartesian dip with planned hover-point-hover motion.
- `54e1dce` constrained hover and dip heights.
- `442c219` restored smooth hover-point-hover motion.
- `f0a6d54`, `0399516`, and `4648ac5` tuned hover behavior.
- `5c5ae3c` improved motion reliability and game retries.
- `3fba614` increased velocity and acceleration scaling.
- `5845378` added optional gripper closing for pointing.

One important lesson was that the motion that looks best in theory is not
always the best on the real robot. We tried Cartesian dipping, but the current
code intentionally avoids `computeCartesianPath()` because the lab robot was
smoother and more reliable with normal MoveIt planned poses.

The final current motion behavior is:

1. Move to a hover pose above the block.
2. Move lower to indicate/point at the block.
3. Move back to hover.
4. Continue with the next block.
5. Return home after the sequence if configured.

## 11. UI Development

The UI was added because running every ROS node manually became inconvenient.
It also helped make the project feel more like a complete game instead of only
a set of terminal commands.

Important UI commits:

- `e0704d1` added a local dashboard prototype.
- Many April 17 commits updated `ui/index.html`.
- `c0f4223` added sequenced startup, progress tracking, and play again.
- `55a8bb8` fixed ROS setup path detection.
- `82371d3` reapplied sound, speech, and feedback logic.
- `92ad11c` polished the player-facing interface.
- `c5d5524` started motion before game from the UI.
- `e2ee52d` added game sound assets.
- `9f30255` sped up UI session startup.

The UI currently contains:

- `ui/server.py`
- `ui/app.js`
- `ui/index.html`
- `ui/styles.css`
- `ui/sounds/`

It can show live game state, score, motion state, and help start the system in
a more controlled order.

## 12. May 15 Vision Experiments And Reverts

May 15 was one of the busiest days in the commit history. A lot of vision
ideas were tried, adjusted, and reverted. This part of the history is important
because it shows the real debugging process.

Important commits:

- `da04dd0` increased the base exclusion radius and added extra exclusions.
- `8cff05d` added exclusions for back-corner and back-edge false detections.
- `b2ce621` rejected far-jump candidates.
- `f8a7987` tried to prevent far color-track jumps.
- `0ab705e` kept color tracking local during active tracks.
- `11ced1f` reverted the local active-track change.
- `4da8d4b` reverted the far-jump prevention change.
- `7287ad8` constrained vision to the table workspace.
- `ba503cd` added image-space ROI masking.
- `d5a52a3` reverted the image-space ROI mask.
- `9ce0f57` widened the lab vision workspace.
- `6d1f3fb` reverted the table workspace constraint.

Then on May 16, commit `c7b8953` reset the vision node to the May 8 baseline.
The commit message explains that the team wanted a clean starting point before
adding image crop work. This kept the useful base LED exclusion but removed the
extra experimental churn.

After that:

- `24ea1dc` added a tunable pre-detection image crop.
- `d84c640` set initial crop bounds for the table play area.

The current approach is:

1. Keep the stable May 8 vision baseline.
2. Keep the base LED exclusion.
3. Crop the image before HSV detection.
4. Tune the crop from `/vision/debug_overlay` in the lab.

This is cleaner than continuously adding special-case exclusions.

## 13. Current Important Files

### Core package

- `CMakeLists.txt`
- `package.xml`
- `msg/Block.msg`
- `msg/BlockArray.msg`
- `msg/BlockSequence.msg`
- `msg/PlayerSelection.msg`

### Vision

- `src/vision_node.cpp`
- `VISION_TUNING.md`
- `scripts/start_camera.sh`
- `scripts/run_vision.sh`

### Player selection

- `scripts/player_selection.py`
- `scripts/test_player_selection.py`

### Game

- `src/game_node.cpp`

### Motion

- `src/motion_node.cpp`
- `newmotion/motion_moveit_node.cpp`
- `newmotion/motion_hw_node.cpp`
- `newmotion/probe_motion_interface.sh`
- `newmotion/MOTION_RUNBOOK.md`

### UI

- `ui/server.py`
- `ui/app.js`
- `ui/index.html`
- `ui/styles.css`
- `ui/sounds/`

### Config and launch

- `config/game_params.yaml`
- `launch/full_system.launch`
- `launch/vision_only.launch`
- `launch/test_rviz.launch`
- `launch/real_robot.launch`

## 14. Current Parameters That Matter

Some of the most important current settings are in `config/game_params.yaml`.

The game uses three reliable colors:

```yaml
disable_red: true
min_detected_blocks_required: 3
```

The current vision crop is:

```yaml
image_crop_x_min: 280
image_crop_x_max: 1000
image_crop_y_min: 400
image_crop_y_max: 660
```

The real robot planning group is:

```yaml
planning_group: "arm"
```

The main game-to-motion topic is:

```text
/target_sequence
```

## 15. What Is Working Now

The current project can:

- detect blocks using RealSense RGB and aligned depth
- publish robot-frame block positions
- use three stable colors in the lab
- detect player selections by missing slots
- generate memory sequences
- send full target sequences to motion
- execute robot pointing through MoveIt
- report motion status back to the game
- show score and state
- run through a browser UI

## 16. Current Limitations

The project still has some limitations:

- Red detection is still not reliable enough for the real lab scene.
- Player selection works, but it is not a perfect gesture detector.
- The crop rectangle still needs final tuning on the lab machine.
- Motion still needs careful real-robot testing under repeated full games.
- Some historical notes still mention `/target_block`, but the current system
  uses `/target_sequence`.

## 17. Lessons Learned

The biggest lesson was that real robot integration changes the project. The
original plan was clean and simple, but the real lab setup introduced problems
that were not obvious at the beginning.

Things we learned:

- A mock system is useful because it lets the team test the architecture early.
- Real camera topics and depth alignment must be checked before tuning colors.
- Debug images are necessary for vision work.
- Red is difficult in real lighting because it overlaps with skin and
  background colors.
- The robot should not receive fallback target positions unless they are safe.
- Motion status needs to be specific, not just "done" or "moving".
- Reverts are part of real development. Several reverted commits helped us get
  back to a stable baseline.
- Runtime configuration was just as important as code. `game_params.yaml` was
  the most frequently changed file in the project.

## 18. Most Changed Files

This gives a good picture of where most of the work happened:

| File | Times touched | Why it changed often |
|---|---:|---|
| `config/game_params.yaml` | 51 | Lab tuning and integration parameters |
| `src/vision_node.cpp` | 36 | Real camera detection and false-positive handling |
| `src/game_node.cpp` | 22 | Game state, sequence flow, motion coordination |
| `newmotion/motion_moveit_node.cpp` | 18 | Real robot MoveIt behavior |
| `ui/index.html` | 15 | UI iteration |
| `CMakeLists.txt` | 12 | Build and message integration |
| `launch/full_system.launch` | 10 | System startup integration |
| `launch/vision_only.launch` | 9 | Vision debugging |
| `src/motion_node.cpp` | 8 | Demo/RViz motion |
| `ui/server.py` | 7 | UI process control |
| `ui/app.js` | 7 | UI game behavior and sounds |
| `scripts/player_selection.py` | 7 | Slot-based player selection |

## 19. Commit Timeline

This timeline is included because many important ideas were later overwritten
or reverted. It shows the actual progress of the project, not only the final
files.

| Date | Commit | What changed |
|---|---|---|
| 2026-02-20 | `453bd35` | Initial ROS1 package with vision, game, motion, messages, launch files, docs |
| 2026-03-02 | `1bf3858` | Improved motion state machine and return-home behavior |
| 2026-03-03 | `268bdd5` | Improved vision node and added tuning docs |
| 2026-03-03 | `d8d9135` | Merged vision tuning branch |
| 2026-03-03 | `048c929` | Replaced game node and added Panda URDF |
| 2026-03-04 | `09291c2` | Improved motion node 3D distance and state machine |
| 2026-03-04 | `fae23fe` | Updated RViz config and simulation launch during testing |
| 2026-03-05 | `e37d119` | Fixed errors and tested node publishing and parameters |
| 2026-03-05 | `09b0dee` | Changed `test_rviz.launch` after test errors |
| 2026-03-05 | `336e122` | Updated game node |
| 2026-03-05 | `4af69e5` | Updated game node again |
| 2026-03-05 | `3464666` | Cleaned launch setup and removed old simulation launch |
| 2026-03-06 | `4b8a207` | Improved vision player selection and vision-only launch |
| 2026-03-06 | `4b1a547` | Made player selection test script executable |
| 2026-03-06 | `1aad75d` | Adjusted RViz test and manual input timing |
| 2026-03-06 | `25dbf4a` | Updated robot IP address in launch file |
| 2026-03-06 | `cb7e919` | Changed real robot launch |
| 2026-03-06 | `2ca7666` | Updated game node |
| 2026-03-06 | `dfe81f1` | Updated game and motion |
| 2026-03-07 | `52eaed9` | Refined launch and vision debugging flow |
| 2026-03-13 | `714b0ac` | Added motion interface probe and hardware motion stub |
| 2026-03-13 | `bc94f58` | Grouped motion helpers under `newmotion` |
| 2026-03-13 | `05ea1ec` | Game waits for motion subscriber |
| 2026-03-13 | `57a6bfc` | Vision defaults changed to `/realsense/*` topics |
| 2026-03-13 | `3c5ba01` | Added MoveIt-backed motion executor |
| 2026-03-13 | `c3eb7fb` | Added ROI/workspace filters for vision false positives |
| 2026-03-19 | `8b86e0c` | Excluded skin mask from block detection |
| 2026-03-19 | `3c1ce39` | Added optional red-disable mode |
| 2026-03-19 | `67d690b` | Hardened lab vision and motion behavior |
| 2026-03-19 | `422823d` | Hardened game, vision, and motion coordination |
| 2026-03-19 | `7b80312` | Rewrote docs for current architecture at that time |
| 2026-03-19 | `4daa250` | Added integration lessons to docs |
| 2026-03-20 | `db2782b` | Synchronized RGB and depth |
| 2026-03-20 | `432472a` | Used buffered depth matching |
| 2026-03-20 | `309bfdd` | Added hand-mask debug and tunable skin HSV |
| 2026-03-20 | `ea3e0e7` | Allowed game to start with three detected blocks |
| 2026-03-27 | `c07788b` | Hardened player selection and updated docs |
| 2026-03-27 | `e0704d1` | Added local dashboard prototype |
| 2026-04-07 | `85a80c3` | Restored stable three-color vision baseline |
| 2026-04-08 | `91e6061` | Hardened vision pipeline and fixed game dispatch |
| 2026-04-10 | `b1e14ff` | Fixed depth/RGB sync blocking debug output |
| 2026-04-10 | `ae163a8` | Fixed zero timestamp sync failure |
| 2026-04-10 | `c1f3681` | Processed depth despite timestamp drift |
| 2026-04-10 | `cb9eb24` | Started RealSense with aligned depth in launch |
| 2026-04-10 | `df0309d` | Added camera startup script |
| 2026-04-10 | `92f28f9` | Published debug image before camera info arrived |
| 2026-04-10 | `4050ca8` | Added `run_vision.sh` |
| 2026-04-15 | `47a5e61` | Added isolated vision experiments |
| 2026-04-16 | `28facd9` | Updated motion node for new movement logic |
| 2026-04-17 | `18eeeeb` | Updated UI HTML |
| 2026-04-17 | `f9e26f8` | Updated UI HTML |
| 2026-04-17 | `1943c33` | Updated UI HTML |
| 2026-04-17 | `cba25e5` | Updated UI HTML |
| 2026-04-17 | `c7b46d0` | Updated UI HTML |
| 2026-04-17 | `cf4613c` | Updated UI HTML |
| 2026-04-17 | `e28d10b` | Updated UI HTML |
| 2026-04-17 | `5128f50` | Updated UI HTML |
| 2026-04-17 | `3cb3af0` | Updated UI HTML |
| 2026-04-17 | `b6767c1` | Updated UI HTML |
| 2026-04-17 | `d3c805d` | Updated UI HTML |
| 2026-04-17 | `99b8bbc` | Updated UI HTML |
| 2026-04-23 | `e696a3a` | Fixed `try_clean` Python ROS integration |
| 2026-04-23 | `036fddc` | Fixed slot selection logic |
| 2026-04-23 | `fd82c33` | Wired slot selection into launch |
| 2026-04-23 | `84dae2f` | Restored lab motion MoveIt behavior |
| 2026-04-23 | `9eb3d00` | Small update |
| 2026-04-24 | `d02cb90` | Simplified always-on player selection |
| 2026-04-24 | `fbf468f` | Removed older ROI/workspace/skin-mask experiments from main vision |
| 2026-04-24 | `881177f` | Removed visualization markers from vision node and docs |
| 2026-04-24 | `a6ea165` | Cleaned up player-selection Python file |
| 2026-04-24 | `9af02b3` | Removed named namespace from vision node |
| 2026-04-24 | `9df9c57` | Added auto-reset when all blocks are removed |
| 2026-04-24 | `7afd2c6` | Cleaned up game parameters |
| 2026-04-24 | `560dffd` | Removed unreachable legacy timer code |
| 2026-04-24 | `4595c81` | Added hold time so robot pauses to show block |
| 2026-04-30 | `a0b6e66` | Added sequence-based game and motion flow |
| 2026-04-30 | `5ac513f` | Increased motion timeout for real robot |
| 2026-05-01 | `c0f4223` | Added UI startup flow, progress tracking, play again |
| 2026-05-01 | `79dd542` | Added planning group `arm` to motion launch command |
| 2026-05-01 | `59e69c4` | Set planning group to `arm`, increased depth age |
| 2026-05-01 | `ae74782` | Excluded robot base LED from vision |
| 2026-05-01 | `55a8bb8` | Detected ROS setup path dynamically |
| 2026-05-01 | `ca85340` | Smoothed Cartesian dip execution |
| 2026-05-01 | `5ec8268` | Set base exclusion radius default |
| 2026-05-05 | `3416b6e` | Replaced Cartesian dip with planned hover-point-hover motion |
| 2026-05-05 | `74d3413` | Refreshed README for demo workflow |
| 2026-05-08 | `be69489` | Added UI sound effects |
| 2026-05-08 | `673f507` | Added more UI sound effects |
| 2026-05-08 | `54e1dce` | Constrained hover and dip heights |
| 2026-05-08 | `959de1d` | Changed game node and parameters to avoid abort/timeout |
| 2026-05-08 | `82371d3` | Reapplied sound, speech, and feedback logic |
| 2026-05-08 | `442c219` | Restored smooth hover-point-hover motion |
| 2026-05-08 | `92ad11c` | Polished player-facing UI flow |
| 2026-05-08 | `f0a6d54` | Changed MoveIt motion node and parameters |
| 2026-05-08 | `0399516` | Adjusted hover lift for robot arm |
| 2026-05-08 | `4648ac5` | Turned off hover lift |
| 2026-05-08 | `150dbb9` | Adjusted vision node so camera detects blocks better |
| 2026-05-15 | `5c5ae3c` | Improved motion reliability and game retries |
| 2026-05-15 | `da04dd0` | Increased base exclusion and added extra exclusions |
| 2026-05-15 | `8cff05d` | Added extra exclusions for false detections |
| 2026-05-15 | `c5d5524` | Started motion before game from UI |
| 2026-05-15 | `e2ee52d` | Added game sound assets |
| 2026-05-15 | `9f30255` | Sped up UI session startup |
| 2026-05-15 | `3fba614` | Increased motion velocity and acceleration scaling |
| 2026-05-15 | `5845378` | Added optional gripper close for pointing |
| 2026-05-15 | `d3c1c0e` | Waited for motion readiness before dispatch |
| 2026-05-15 | `b2ce621` | Rejected far-jump vision candidates |
| 2026-05-15 | `f8a7987` | Tried to prevent far color-track jumps |
| 2026-05-15 | `0ab705e` | Kept color tracking local during active tracks |
| 2026-05-15 | `11ced1f` | Reverted local active-track change |
| 2026-05-15 | `4da8d4b` | Reverted far-jump prevention |
| 2026-05-15 | `7287ad8` | Constrained vision to table workspace |
| 2026-05-15 | `e1751ab` | Stabilized motion defaults and target filtering |
| 2026-05-15 | `ba503cd` | Added image-space ROI mask |
| 2026-05-15 | `d5a52a3` | Reverted image-space ROI mask |
| 2026-05-15 | `9ce0f57` | Widened lab vision workspace |
| 2026-05-15 | `6d1f3fb` | Reverted table workspace constraint |
| 2026-05-16 | `c7b8953` | Reverted vision to May 8 baseline before crop work |
| 2026-05-16 | `24ea1dc` | Added tunable pre-detection image crop |
| 2026-05-16 | `d84c640` | Set initial crop bounds for table area |

## 20. Final Summary

This project started as a simple ROS1 memory game skeleton and became a real
robot system through many small integration steps. The current project uses
RealSense vision, slot-based player selection, sequence-based game logic,
MoveIt robot motion, and a browser UI.

The most important changes were not only new features. A lot of progress came
from fixing problems found in the lab: wrong camera topics, RGB/depth sync,
false detections, red color instability, motion planning behavior, and unsafe
assumptions in the game node. The commit history shows that the project was
built by testing, failing, reverting, and improving until the system became
usable on the real robot.
