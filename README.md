# Navigation Onboarding 2026

Welcome to the navigation sub-team! This onboarding project drops you into a stripped-down version of Maverick, our actual robot code monorepo, with all the same tooling you'll use on real tickets. You'll build one real piece of the nav stack: a node that turns raw encoder velocity into odometry.

## 1. What You're Building

Every robot needs to know where it is. Wheel encoders give you a velocity — how fast you're moving and turning right now — but nothing tracks *position* on its own. Your job is to write a node that:

- Subscribes to `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`)
- Integrates that velocity over time into a running estimate of position and heading
- Publishes the result as `odom` (`nav_msgs/Odometry`) and broadcasts the corresponding `odom` → `base_link` TF
- Exposes a service to reset that estimate back to the origin

This is a real piece of the localization stack — the same shape of node runs on the actual robot, just simplified so you can validate it in simulation before ever touching hardware.

## 2. Repo Structure

```
src/
  bringup/            launch files and shared config (frames, RViz) for the whole stack
  core/utils/          shared library: config loading, TF/geometry helpers, node lifecycle
  description/         robot URDF (for RViz visualization only, not simulated physics)
  simulation/
    enc_vel_mock_publisher/   drives a live square trajectory, publishes enc_vel
  template/           package scaffolds used by `just create-pkg` (don't edit directly)
bags/                 recorded rosbags used for validation (see §13)
```

You'll add your own package under `src/localization/` (mirroring where this node lives in the real monorepo).

## 3. Environment Setup

```
just setup    # installs pixi environment, shell completions
just build    # builds the workspace once to confirm everything compiles
```

If `just build` succeeds with no packages of your own yet, your environment is good to go.

## 4. Create Your Branch

```
git checkout -b <your-name>/enc-odom-publisher
```

All work happens on this branch; your submission is the PR it opens against `main` (see §16).

## 5. Creating a Package

Use the scaffolding tool rather than hand-writing boilerplate:

```
just create-pkg src/localization enc_odom_publisher
```

This copies `src/template/template_python` into `src/localization/enc_odom_publisher`, renaming everything to your package name, and regenerates `pyrightconfig.json` so your editor picks up the new package. Take a look at what got generated before writing any code — it's the same shape every package in this repo follows.

The scaffold leaves several `TODO` placeholders that aren't code, so nothing will fail to build if you forget them — fill them in anyway, since a PR reviewer will expect it:

- `package.xml`'s `<description>` and `setup.py`'s `description=` (currently `"TODO: Package description"`)
- `README.md`'s summary line and Subscribed/Published Topics tables
- Your config dataclass's docstring `Attributes:` list (§6) — document each field as you add it, not at the end

## 6. The Config Pattern

Every node in this repo loads its tunable parameters through a frozen dataclass, not raw `declare_parameter`/`get_parameter` calls:

```python
@dataclass(frozen=True)
class EncOdomPublisherConfig:
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        # raise ValueError on invalid combinations
        ...
```

```python
self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)
```

`utils.config.load` declares a ROS 2 parameter per field, reads any values passed in via launch, and falls back to the dataclass defaults otherwise. See `src/core/utils/utils/config.py` for the full mapping rules (nested dataclasses, lists, `Literal` types, etc.) — you won't need most of it for this exercise, but you will see it again on real packages.

## 7. Publisher / Subscriber Basics

Your node subscribes to `enc_vel` and publishes `odom`:

```python
self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
```

Note both topic names are generic — `enc_vel`, not `enc_vel/raw`. Launch files own the actual wiring (§11); your node shouldn't hardcode where its input comes from.

## 8. The Odometry Math

`enc_vel` gives you linear velocity `vx` and angular velocity `wz` in the robot's own frame. To get position, integrate:

```
dt = time since last enc_vel message
mid_heading = heading + 0.5 * wz * dt      # midpoint method, more accurate than naive Euler
x += vx * dt * cos(mid_heading)
y += vx * dt * sin(mid_heading)
heading += wz * dt
```

Using the midpoint of the heading during the timestep (rather than the heading at the start or end of it) noticeably reduces integration error for a given `dt`, especially while turning. `utils.geometry`'s `Point2d`/`Rotation2d` give you rotation composition for free if you want to use them instead of raw trig.

Guard against bad input: drop updates where `dt <= 0` or `dt` is implausibly large (e.g. after a pause), and validate that the message's `frame_id` matches your configured `base_frame_id`.

## 9. Broadcasting TF

Publish the `odom` → `base_link` transform alongside `Odometry`, using the same position/heading estimate:

```python
self.tf_broadcaster = TransformBroadcaster(self)
...
self.tf_broadcaster.sendTransform(TransformStamped(
    header=Header(stamp=now, frame_id=self.config.odom_frame_id),
    child_frame_id=self.config.base_frame_id,
    transform=Transform(translation=..., rotation=...),
))
```

This isn't optional plumbing — RViz's RobotModel display and the Odometry display both need this transform to place anything relative to the fixed frame. Without it, the robot mesh stays frozen at the origin while `odom` reports it moving.

## 10. The Reset Service

Add a service (`std_srvs/Trigger` is sufficient) that zeroes your position/heading estimate back to `(0, 0, 0)`. Two things people commonly miss:

- Resetting state doesn't automatically publish anything — make sure the *next* TF broadcast reflects the reset immediately, not just the next scheduled timer tick, so downstream visualization doesn't briefly show a stale pose.
- This is the easiest way to verify your integration math resets cleanly: reset, drive the square (§12), and confirm it starts from the same place every time.

## 11. Adding Your Node to Launch Files

Edit `src/bringup/launch/localization.launch.py` — it's an empty template with a commented-out example showing the expected shape (parameters loaded from `frames.yaml`, and the `enc_vel` → `enc_vel/raw` remap mentioned in §7):

```python
Node(
    package="enc_odom_publisher",
    executable="enc_odom_publisher",
    name="enc_odom_publisher",
    output="screen",
    parameters=[
        {"odom_frame_id": frames["odom_frame"]},
        {"base_frame_id": frames["base_frame"]},
    ],
    remappings=[("enc_vel", "enc_vel/raw")],
),
```

`src/bringup/launch/simulation.launch.py` starts `enc_vel_mock_publisher` — you shouldn't need to touch it.

## 12. Running Against the Mock Publisher

Like the rosbag flow below, this is three separate launches — `core.launch.py` and `localization.launch.py` bring up the robot description and your node, and `simulation.launch.py` starts the mock publisher feeding them:

```
just build
ros2 launch bringup core.launch.py
ros2 launch bringup localization.launch.py
# in a third terminal:
ros2 launch bringup simulation.launch.py
```

`enc_vel_mock_publisher` drives a repeating square with a small constant drift bias and noise (see its README for config). A correct node traces a mostly-closed square that's slightly offset by the time it comes back around — a perfectly closed loop with zero drift usually means the noise/bias isn't making it through your integration at all, which is itself worth double-checking.

## 13. Running Against the Rosbag

`bags/` contains a recorded rosbag of a heart-shaped trajectory (driven via joystick teleop through the real sensor pipeline, so it carries realistic noise). Launch your node without the mock publisher, then play the bag in a second terminal:

```
ros2 launch bringup core.launch.py
ros2 launch bringup localization.launch.py
# in a second terminal:
ros2 bag play bags/<heart-bag-name>
```

## 14. Visualizing in RViz2

We use RViz2 directly, launched manually (not baked into a launch file). Launch it plain, then load the saved config via **File → Open Config**, browsing to `src/bringup/rviz/onboarding.rviz` in the repo:

```
rviz2
```

The provided config sets the fixed frame to `odom` and has RobotModel, TF, and Odometry displays already added.

## 15. Tooling: Formatting & Linting

```
just format   # auto-formats all source files
just lint     # runs the same checks CI runs
```

Run both before opening your PR — CI enforces them, and fixing lint failures after the fact is more annoying than running `just format` up front.

## 16. Opening Your PR

Push your branch and open a PR against `main`. The PR template will ask for:

- A screenshot of RViz showing a valid trajectory against the mock publisher (square)
- A screenshot of RViz showing a valid trajectory against the rosbag (heart)

CI must pass (build + test + lint) before your PR is considered complete.
