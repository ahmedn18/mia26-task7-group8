# MIA26 — Task 7.2, Group 8

Autonomous maze navigation for a TurtleBot3 Burger in Gazebo Harmonic, built on ROS 2
actions and the maze's wall service.

Supervisors: Hamza & Bahig.

## The idea

Two action servers do all the motion, and a single high-level node chains them into a run:

- **`move_x`** — linear movement. Goal is a signed distance in metres.
- **`move_yaw`** — rotation. Goal is a signed angle in radians (+ is left).
- **`solve_maze`** — the orchestrator. Owns the whole sequence and calls the stages in order.

Both action servers publish to `/cmd_vel`, track position on `/odom`, stream feedback
(remaining distance/angle) back to `solve_maze`, and succeed only once the target is
reached. They abort if `/odom` goes silent and on timeout.

The three stages are **classes inside `solve_maze`**, not separate nodes. A stage never
talks to the next stage — the orchestrator calls each one in order and checks its return
value, so ordering and failure handling live in one place.

Each stage calls the wall service itself. Stages never publish to `/cmd_vel` directly;
all motion goes through the two action clients.

## The maze

Everything below is measured from `maze_world.sdf` in the
[simulation repo](https://github.com/eng-Aly/MIA26_phase2_ros_contest).

Arena is 5.0 × 3.0 m. The robot starts at **(0.5, 0.5) facing +x**.

| Feature | Where |
| --- | --- |
| Gate 1 | y = 1.0, gap x ∈ [0.1, 0.9] — **closed** at startup |
| Gate 2 | y = 2.0, gap x ∈ [0.15, 0.85] — **open** at startup |
| Exit | east side, open only for y ∈ [2, 3] |
| Finish pad | x ∈ [4.7, 5.3], y ∈ [1.95, 3.0], centre (5.0, 2.475) |

### The gates

One service moves both: **`/toggle_walls_1_2`**, type `std_srvs/srv/SetBool`.

| Request | Gate 1 | Gate 2 |
| --- | --- | --- |
| `data: true` | opens | closes |
| `data: false` | closes | opens |

Two consequences that shape the whole design:

1. The gates are mutually exclusive, so the robot must be **completely past gate 1**
   before anything requests gate 2 — otherwise gate 1 comes down on it.
2. The service returns `success: true` as soon as the command is *published*, not when
   the wall has finished moving. The wall travels 1.0 m at a 0.5 m/s joint limit, so
   budget `GATE_OPEN_S ≈ 2 s`.

Because of (2), each stage requests its gate as its **first action**, drives its approach
leg, then holds short of the gate for whatever is left of `GATE_OPEN_S` before crossing.
If the drive took longer than the wall did, the hold costs nothing.

### Route

| Stage | Owner | Legs |
| --- | --- | --- |
| 1 | Dina | request gate 1 → yaw +90° → +y 0.30 → hold → +y 0.45 through gate 1, ending at y ≈ 1.25 |
| 2 | Sofian | request gate 2 → +y 0.55 → hold → +y 0.70 through gate 2, ending at y ≈ 2.5 |
| 3 | Basmala | yaw −90° → +x 4.5 onto the finish pad |

About 6.5 m of driving plus two 90° turns. Distances are starting values from the world
file — tune them, but flag it if you move a handoff point, since the next stage starts
from wherever you stopped.

## Environment

ROS 2 Jazzy on Ubuntu 24.04, Gazebo Harmonic. Follow the simulation repo's README to
install `ros-jazzy-ros-gz` and the TurtleBot3 packages, then clone it into the same
workspace:

```bash
cd ~/training_ws/src
git clone https://github.com/eng-Aly/MIA26_phase2_ros_contest.git
git clone https://github.com/ahmedn18/mia26-task7-group8.git
cd ~/training_ws && colcon build && source install/setup.bash
```

Notes worth knowing before you start:

- `/cmd_vel` is bridged as plain `geometry_msgs/msg/Twist`. TwistStamped messages are
  not bridged and will silently do nothing.
- `/odom` (`nav_msgs/msg/Odometry`) is already published by the simulation.
- `/clock` is bridged, so run our nodes with `use_sim_time: true`.
- The run timer starts when the simulation launches, not when the solver starts, and the
  simulation shuts itself down the moment the robot touches the finish pad.
- Don't change the `robot_name` launch argument — the finish trigger only fires for a
  model named `burger` or `robot`.

## Working on this

`main` is the submission branch. Work happens on `dev`.

Branch per task (`feat/move-x`, `feat/move-yaw`, `feat/stage-1`, …), small logical
commits, then a PR into `dev`. Please don't push a finished package as one commit —
version control is graded directly.

## Team

| Member | ID | Owns |
| --- | --- | --- |
| Ahmed Nader | E465 | repo, integration, `solve_maze`, action definitions |
| Mohamed Samir | E585 | `move_x` action server |
| Masa Mostafa | E962 | `move_yaw` action server |
| Mariam Fakhry | E180 | `move_yaw` action server |
| Dina Zaghloul | E430 | stage 1 |
| Sofian | — | stage 2 |
| Basmala Essam | E338 | stage 3 |
