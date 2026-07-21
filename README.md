# Navigation Onboarding 2026

Welcome to the navigation sub-team! This onboarding project lets you learn key concepts you'll need to succeed as a nav member with a simplified version of Maverick, our actual robot code monorepo*, with all the same tools you'll use on real projects. You'll rebuild a simple but crucial piece of the nav stack: a ROS node that turns raw encoder velocity into odometry.
*monorepo: a single storage repository for one large project (https://en.wikipedia.org/wiki/Monorepo)
## 1. What You're Building

Every robot needs to know where it is. Your job is to write to calculate odometry, or estimated position based on encoder data, for Maverick. Each motor has an encoder that tracks the movement of its wheel, and from this data, we can calculate the vector of Maverick's current motion. This velocity data can be integrated over time to get an estimate of Maverick's current position. In order to take input from the motor encoders, calculate the odometry, and send it to the navigation algorithms, we use a ROS node.

This odometry node must:
- Subscribe to `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`)
- Integrate that velocity over time into a running estimate of position and heading
- Publish the result as `odom` (`nav_msgs/Odometry`) and broadcasts the corresponding `odom` → `base_link` TF (transform)
- Provide a service that can be accessed by Maverick's other systems to zero the odometry.

Again, you're recreating an actual piece of the localization stack; when you've completed the project, we encourage you to compare for yourself. A few features have been removed so that it's easier to build and understand, but in theory it could still be a functional odometry system for Maverick.

## 2. Repo Structure

```
src/
  bringup/            launch files and shared config (frames, RViz) for the whole stack
  core/utils/          shared library: config loading, TF/geometry helpers, node lifecycle. Basically, our imports and custom data types/math functions.
  description/         robot URDF (A file that represents the physical model of our robot for use in RViz visualization, but not physics simulation. https://en.wikipedia.org/wiki/URDF)
  simulation/
    enc_vel_mock_publisher/   drives a live square trajectory, publishes enc_vel
  template/           package scaffolds used by `just create-pkg` (don't edit directly)
bags/                 recorded rosbags used for validation (see §13)
```
In general, these are the only parts of the repo you need to worry about (unless something is very very wrong, and if so it's probably not your fault). You'll add your own package under `src/localization/` (mirroring where this node lives in the real monorepo).


## 3. Cloning the Repo
If you scroll to the top of this Github page, you'll see a green button with the word "Code" on it. Click it, and under HTTPS, hit the copy symbol for the link shown. Then, open VSCode or your terminal.

### VSCode:
- Hit the "Clone Git Repository" button.
- Paste in the link and follow the prompts. Be sure to say that you trust the authors!

### Terminal:
- Navigate to a folder of your choosing.
- Type 'git clone {the link you copied}' and hit enter.
- Open the folder in VSCode or an IDE of your choosing.

You'll also want to work on *your own branch.* To do this, run git  
## 4. Environment Setup

```
just setup    # installs pixi environment, shell completions
just build    # builds the workspace once to confirm everything compiles
```

If `just build` succeeds with no packages of your own yet, your environment is good to go.

## 5. Create Your Branch
Open a terminal in VSCode or the terminal app in this project's folder. Paste and run the following command.

```
git checkout -b <your-name>/enc-odom-publisher
```

This creates a separate branch for your work to happen on without it appearing in everyone else's work.

## 6. Creating a Package

The different functions of Maverick are all stored in modules called packages. Packages basically all follow the same template of a bunch of "support/infrastructure" files, with the actual functionality you create only being located in the 'name.py' and 'name_config.py' files. Instead of creating all of that yourself it, run this scaffolding command:

```
just create-pkg src/localization enc_odom_publisher
```

This copies `src/template/template_python` into `src/localization/enc_odom_publisher`, renaming everything to your package name, and regenerates `pyrightconfig.json` so your editor picks up the new package. Take a look at what got generated before writing any code — it's the same shape every package in this repo follows.

The scaffold leaves several `TODO` placeholders that aren't code, so nothing will fail to build if you forget them — fill them in anyway, since a PR reviewer will expect it:

- `package.xml`'s `<description>` and `setup.py`'s `description=` (currently `"TODO: Package description"`)
- `README.md`'s summary line and Subscribed/Published Topics tables
- Your config dataclass's docstring `Attributes:` list (§7) — document each field as you add it, not at the end
- Your information in setup.py and package.xml (so we know who made what)

## 7. The Config Pattern

Every node in this repo loads its tunable parameters through a frozen dataclass, not runtime declarations and access calls. This way, they can't be changed in runtime, and when we want to adjust the parameters on a package, they're all organized in one place:

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

`utils.config.load` declares a ROS 2 parameter per each field of the selected config file, reads any values passed in via launch, and uses  dataclass defaults otherwise. See `src/core/utils/utils/config.py` for the full mapping rules (nested dataclasses, lists, `Literal` types, etc.) — you won't need most of it for this exercise, but you will see it again on real packages.

## 8. Publisher / Subscriber Basics

Your node subscribes to the `enc_vel` topic and publishes to the `odom` topic:

```python
self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
```

Note: If you were to run the Maverick stack and open RViz or use the echo list command to see all the active topics, you'd see both an `enc_vel` and `enc_vel/raw` topic. We want the node to use `enc_vel`, and here's why : say we discovered something wrong with our encoder velocity--maybe its data needs to be filtered. Normally, 'enc_vel/raw' publishes right to 'enc_vel', but by adding a filter between the topics, every node that needs velocity can get it without needing to change which topic it's subscribed to. In that case, if the odom node were subscribed to 'enc_vel/raw', it would be getting the unfiltered data. 
For more review on publishers and subscribers, review the slides.

## 9. The Odometry Math

`enc_vel` gives you linear velocity `vx` and angular velocity `wz` in the robot's own frame of reference. To get position, integrate:

```
dt = time since last enc_vel message
mid_heading = heading + 0.5 * wz * dt      # midpoint method, more accurate than naive Euler's Method
x += vx * dt * cos(mid_heading)
y += vx * dt * sin(mid_heading)
heading += wz * dt
```

By using the midpoint of the initial and final headings of each timestep, we can get a more reliable estimate of the direction the robot moved over the last timestep, especially while turning. `utils.geometry`'s `Point2d`/`Rotation2d` can be used to convert rotation and positions without you needing to worry about the trigonometry, but it is importnt to understand the mechanics of odometry.

It's also good practice to make sure your node is protected from bad input. For starters, drop the message (ie not calculate odometry updates) when the subscriber's received 'dt' is less than or equal to 0 or is longer than say, one second (as would happen if comms were lost). It's also good to check that each message's `frame_id` matches the `base_frame_id` from the config file.

## 10. Broadcasting TF

ROS's Transform, or TF2 (https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Tf2.html), library is used to translate between different frames of reference. 

### What are Frames of Reference?
Say you tell the robot to drive straight forward (+x). Next, you tell the robot to turn around 180 degrees, then drive forward again. If you drew this motion on a map, the robot has just driven out to a point (+x) and gone back to the start (-x), but from the robot's prespective, it only moved forward (+x), turned, and moved forward again. Frames of reference tell us how to get that motion on the world map from the robot's movements and rotations relative to it's current position and vice versa. In other words, frames of reference are used to track the difference between the origins and 0-degree angles of two planes or objects (In this case the world map and the robot).

Paste in this code and fill in the 'transform=' correctly.

```python
self.tf_broadcaster = TransformBroadcaster(self)
...
self.tf_broadcaster.sendTransform(TransformStamped(
    header=Header(stamp=now, frame_id=self.config.odom_frame_id),
    child_frame_id=self.config.base_frame_id,
    transform=Transform(translation=..., rotation=...),
))
```

In terms of how this code is actually used, RViz's RobotModel display and the Odometry display need it to move the robot model. Otherwise, the model stays at the origin even though `odom` tracks its actual motion.

## 11. The Reset Service

Add a service (`std_srvs/Trigger` is sufficient) that zeroes your position/heading estimate back to `(0, 0, 0)`. This lets other ROS nodes request that the odometry node zero itself. 

For this one, we aren't giving you the starter code — go look at the slides, or better yet, the ROS2 docs or the main Maverick repo, to see if you can figure it out for yourself.

To test this, the easiest thing to do is to reset, drive the square (§13), and confirm it starts from the same place every time. It'll also make general testing easier.

A common error to watch out for: Resetting state doesn't automatically publish anything, so you need to make sure the next *TF broadcast* reflects the reset immediately, not just the next scheduled timer tick, so downstream visualization doesn't briefly show a stale pose.


## 12. Adding Your Node to Launch Files

In order for all of our nice, simple, build and bringup commands to work, we need to set up some infrastructure first. This is honestly pretty boring, and they look basically the same for each package. Every package (§7) has its own launch.py file, which basically connects a package name with an executable file and the frames of reference it needs, and renames (remaps) any ROS topics it wants to use under a different name (In this case, the 'enc_vel' vs. 'enc_vel/raw' thing from (§8)). For example, the simulation package's `src/bringup/launch/simulation.launch.py` (already completed for you) starts `enc_vel_mock_publisher`.

Here's the launch code for our node. Paste it into the launch.py file of your localization folder.

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


## 13. Running Against the Mock Publisher

To run the simplified nav stack, you need to launch three separate packages. Launching `core.launch.py` brings up/starts the core robot functionality, `localization.launch.py` starts the node you just made, and `simulation.launch.py` starts the demo encoder simulation that publishes to the 'enc_vel/raw' topic:

```
# in a first terminal:
just build
ros2 launch bringup core.launch.py
# in a second terminal:
ros2 launch bringup localization.launch.py
# in a third terminal:
ros2 launch bringup simulation.launch.py
```

`enc_vel_mock_publisher` publishes the encoder data for a repeating square with a small constant drift bias and noise (see its README for config details). If your node traces a mostly complete square that goes back to roughly the starting point, it's probably working. The square won't be perfect because of the aforementioned noise.

## 14. Running Against the Rosbag

`bags/` contains a recorded rosbag of a heart-shaped trajectory (driven via joystick teleop through the real sensor pipeline, so it also has noise). Launch the robot stack like you did in (§13), but instead of launching the simulation package, use the following ROS command to replay the previously recorded ROS bag.

```
ros2 bag play bags/<heart-bag-name>
```

Replaying ROS bags is a valuable tool for us, because it allows us to test our algorithms and visualization tools against known sets of input that either replay or closely replicate actual robot conditions. If we know that the ROS bag represents driving in a heart, and your node doesn't show that, we can tell it needs to be debugged or tuned.

## 15. Visualizing in RViz2

RViz is a useful tool for quickly visualizing different positional data topics in Maverick's stack. Rather than echoing a topic and reading the output, RViz lets us see things like CV output, intended path, and for our purposes, odometry in a simulated space.
We launch RViz2 directly rather than through a launch file: 

```
rviz2
```
If you have a certain set of topics you regularly want to look at togehter, you can save them as a custom configuration. For this project, use RViz's **File → Open Config** and select `src/bringup/rviz/onboarding.rviz` in the onboarding repo, which sets the fixed frame to `odom` and has RobotModel, TF, and Odometry displays already added.


## 16. Tooling: Formatting & Linting
We also have some formatting commands to make sure your packages match our preferred format:

```
just format   # auto-formats all source files
just lint     # runs the same checks CI runs
```

*Run both before proceeding.* 

## 17. Opening Your PR

Commit your changes, and push to and publish your branch. Open it in Github, and open a Pull Request against `main` to merge your changes. The PR template will ask for:

- A screenshot of RViz showing a valid trajectory against the mock publisher (square)
- A screenshot of RViz showing a valid trajectory against the rosbag (heart)

CI (Continuous Integration checks) must pass (build + test + lint) before your PR is considered complete.

https://www.kern-it.be/en/definitions/pull-request/
