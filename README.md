# Navigation Onboarding 2026

Welcome to the navigation subteam! In this onboarding project, you'll learn the key concepts you need to know as a nav member with a simplified version of Maverick, our actual robot code monorepo*, with most of the same tools you'll use on real projects. In particular, you'll rebuild the odometry ROS node. While simple, encoder odometry is widely used in robotics to track position, so it's very helpful to know!

**Please note** that you are expected to struggle with this project (not too much, we aren't sadists). ROS types in particular can be very confusing, as they nest and interact in confusing ways, especially as they relate to services, and a large part of the challenge of this project is figuring out all of the associated errors. Both Caitlyn and Hannah spent a while debugging their own solutions, so don't feel dumb. We want you to learn how to find examples and details about types from the ROS Docs other online sources, and don't be afraid to ask for help from the leads or each other. Just don't copy or use AI to write your code, you're doing yourself a disservice. Remember, your most important job here is to learn.

*monorepo: a single storage repository for one large project (https://en.wikipedia.org/wiki/Monorepo)
## 1. What is Odometry?

Every mobile robot needs to know its location. Your job is to write to calculate odometry, or estimated position based on encoder data, for Maverick. Each motor has an encoder that tracks its rotation, and from this data, we can calculate Maverick's current velocity vector. These velocity data can be integrated over time to get an estimate of Maverick's current position. In order to take input from the motor encoders, calculate the odometry, and send it to the navigation algorithms, we use a ROS node.

The odometry node must:
- Subscribe to `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`)
- Integrate that velocity over time into a running estimate of position and heading
- Publish the result as `odom` (`nav_msgs/Odometry`) and broadcasts the corresponding `odom` → `base_link` TF (transform)
- Provide a service that can be accessed by Maverick's other systems to zero the odometry.

In this project, you're recreating an actual piece of Maverick's localization stack. You can check for yourself when you've completed the project. A few features have been removed so that it's easier to build and understand, but in theory it could still be a functional odometry system for Maverick.


## 2. Repo Structure

```
src/
  bringup/            launch files and shared config (frames, RViz) for the whole stack
  core/utils/          shared library: config loading, TF/geometry helpers, node lifecycle. Basically, our imports and custom data types/math functions.
  description/         robot URDF (A file that represents the physical model of our robot for use in RViz visualization, but not physics simulation. https://en.wikipedia.org/wiki/URDF)
  simulation/
    enc_vel_mock_publisher/   drives a live square trajectory, publishes enc_vel
  template/           package templates used by `just create-pkg` (don't edit directly!!!)
bags/                 recorded rosbags used for validation (see §14)
```
In general, these are the only parts of the repo you need to worry about (unless something is very very wrong, and if so it's probably not your fault). You'll add your odometry node as a package under `src/localization/` (which is where this node is located in Maverick's monorepo). You'll learn more about packages in a later step, but first you need to do some setup.


## 3. Cloning the Repo
If you scroll to the top of this Github page, you'll see a green button with the word "Code" on it. Click it, and under HTTPS, hit the copy symbol for the link shown. Then, open VSCode or your terminal.

### VSCode:
- Hit the "Clone Git Repository" button.
- Paste in the link and follow the prompts. Be sure to say that you trust the authors!

### Terminal:
- Navigate to a folder of your choosing.
- Type 'git clone <the link you copied>' and hit enter.
- Open the folder in VSCode or an IDE of your choice.

## 4. Environment Setup
Now that you have the repo open in your IDE, open a terminal window in the repo.

```
cd ~/<replace with your file path>/nav-onboarding-2026
just setup    # installs pixi environment, shell completions
just build    # builds the workspace once to confirm everything compiles
```

If `just build` succeeds with no packages of your own yet, your environment has been set up successfully. If not, talk to Caitlyn or Hannah.
Once you do this, you should notice that whenever you open a new terminal in your repo, it should switch to a pixi terminal after a couple seconds.

## 5. Create Your Branch
Open a terminal in VSCode or the terminal app in this project's folder. Paste and run the following command.

```
git checkout -b <your-uniqname>/enc-odom-publisher
```

This creates a separate branch for your work to happen on without it appearing in everyone else's work.

## 6. Creating a Package

The different functions of Maverick are all stored in modules called packages. Packages basically all follow the same template of a bunch of "support/infrastructure" files, with the actual functionality you create only being located in the '<name>.py', '<name>_config.py', and often (but not here) '<name>_impl.py'* files. Instead of having to create all of that yourself, just run this command:

```
just create-pkg src/localization enc_odom_publisher
```

This copies `src/template/template_python` into `src/localization/enc_odom_publisher`, renaming everything to match your package name, and regenerates `pyrightconfig.json` so your editor recognizes the new package. Before writing any code yourself, take a minute to familiarize yourself with the premade format. Every package in this repo follows the same format.

The format leaves several `TODO` placeholder comments. It's good to fill them anyway, since a PR reviewer will expect it:

- `package.xml`'s `<description>` and `setup.py`'s `description=` (currently `"TODO: Package description"`)
- `README.md`'s summary line and Subscribed/Published Topics tables
- Your config dataclass's docstring `Attributes:` list (§8) — document each field as you add it, not at the end
- Your information in setup.py and package.xml (so we know who made what)

*'<name>_impl.py' is used by our more complicated nodes, like goal selection, that require a lot of functionality. This would be very unwieldy to create in the node itself, so we use an 'impl' for readability.

## 7. Adding Your Node to Launch Files

In order for all of our nice, simple, build and bringup commands to work, we need to set up some infrastructure first. This is honestly pretty boring, and they look basically the same for each package. Every package (§6) has its own launch.py file, which basically connects a package name with an executable file and the frames of reference it needs, and renames (remaps) any ROS topics it wants to use under a different name (In this case, the 'enc_vel' vs. 'enc_vel/raw' thing from (§9)). For example, the simulation package's `src/bringup/launch/simulation.launch.py` (already completed for you) starts `enc_vel_mock_publisher`.

Here's the launch code for our node. Paste it into the launch.py file that corresponds to the localization node.

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

*Important tip before proceeding:*
As you go, it's important to be checking your work. Now that the launch file has been created, you can follow the instructions in 
step (§13 and §14) to build and run the stack, which will be essential for debugging. It's also often useful to temporarily add logging messages to trace errors:
```python 
self.get_logger().info("useful info here ")
self.get_logger().info(f"useful var here {<my_float_var>}")
```
Caitlyn's recommendation is to print the function or check that the logging call is in to make sure the node is correctly reaching all the functionality it needs during operation. Rather than printing the line number, print what needed to happen to reach that point in the code, because then you'll understand what isn't happening if that statement is never reached. It's also helpful to print variables to check that everything is being processed correctly; an example for printing a float is shown above.

## 8. What is the Config File?

Every node in this repo loads its tunable parameters through a pre-made class, not runtime declarations and access calls. This way, they can't be changed in runtime, and when we want to tune the parameters on a node/package, they're all organized in one place:

```python
@dataclass(frozen=True)
class EncOdomPublisherConfig:
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        # raise ValueError on invalid combinations
        ...
```
Add the frame_id assignments seen above to the config. The rest is already added.
```python
self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)
```

`utils.config.load` declares a ROS 2 parameter for each field of the selected config file, reads any values passed in via the 'launch' command, and uses the config class's defaults otherwise. See `src/core/utils/utils/config.py` for the full mapping rules (nested dataclasses, lists, `Literal` types, etc.). You won't need it for onboarding, but it shows up in the actual repo.

## 9. Publisher / Subscriber Basics

Your node needs to subscribe to the `enc_vel` topic, which is of type TwistWithCovarianceStamped, and publishes to the `odom` topic, which is of type Odometry. Use the format found there to add them to the init. 

```python
self.create_subscription(<data_type>, "<topic name>", self.<callback_name>, 10)
self.<publisher_name> = self.create_publisher(<data_type>, "<topic_name>", 10)
```

Note: If you were to run the Maverick stack and open RViz or use the echo list command to see all the active topics, you'd see both an `enc_vel` and `enc_vel/raw` topic. We want the node to use `enc_vel`, and here's why : say we discovered something wrong with our encoder velocity--maybe its data needs to be filtered. Normally, 'enc_vel/raw' publishes right to 'enc_vel', but by adding a filter between the topics, every node that needs velocity can get it without needing to change which topic it's subscribed to. In that case, if the odom node were subscribed to 'enc_vel/raw', it would be getting the unfiltered data. 
For more review on publishers and subscribers, review the slides.

## 10. Odometry Calculations

The `enc_vel` subscription gives you linear velocity `vx` and angular velocity `wz` in the robot's frame of reference. We want the node to calculate and publish the robot's global position based on this. Thus, whenever data is received on the subsciber, the node should perform this integration:

```
dt = time since last enc_vel message
mid_heading = heading + 0.5 * wz * dt      # midpoint method, more accurate than basic Euler's Method
x += vx * dt * cos(mid_heading)
y += vx * dt * sin(mid_heading)
heading += wz * dt
```
...and then publish the transformed output. However, there are some situations where we don't want to publish. For starters, drop the message (i.e. don't calculate or publish odometry updates) when the subscriber's received 'dt' is less than or equal to 0 or is longer than say, one second (as would happen if comms were lost). It's also good to check that each message's `frame_id` matches the `base_frame_id` from the config file. You also need to think back to the slides to decide when to publish your data. There are two main ways of doing this, one simpler to implement but less robust, and the other more complex but more like what we do on the main robot.

*Why midpoint method?*
By using the midpoint of the initial and final headings of each timestep in our calculations, we can get a more reliable estimate of the direction the robot moved over the last timestep, especially while turning. `utils.geometry`'s `Point2d`/`Rotation2d` can be used to convert rotation and positions without you needing to worry about the trigonometry, but it is importnt to understand the mechanics of odometry.

*A note on ROS2 types:*
As discussed briefly in the slides, ROS2 uses lots of custom message types. You'll be interacting with a couple of them in this step, including Twist and Time. The most complicated of these is the Odometry type. The template for it is below. For the rest of the message types, you are encouraged to look them up in the ROS2 documentation.
```python
            Odometry(
                header=Header(<look this up!>),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(<look this up!>,),
                    covariance=self.pose_covariance,
                ),
                twist=Twist(<look this up!>),
            )
```
*Pose*
Rotations and positions are by default 3d in ROS. For convenience, you can use Rotation2d and Point2d types instead, but when you publish them, you'll need to use our .to_ros() helper function to covert them to 3d.

*Time*
You will need to look this one up. Two things to note: 
1. Getting 'dt' is not be as simple as subtracting two 'Time' objects. You need to do some preprocessing of your own, which you will need to look up.
2. What happens when you try to get 'dt' when there is no previous time?

*Covariance*
Don't worry too much about this. Covariance is basically a matrix used to represent uncertainty in a pose. For this purpose, our uncertainty is very small (but it must be nonzero), and we only need to set values for x and y axes and the z rotation, which are on the main diagonal of the matrix.

## 11. Broadcasting TF

ROS's Transform, or TF2 (https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Tf2.html), library is used to translate between different frames of reference. 

### What are Frames of Reference?
Say you tell the robot to drive straight forward (+x). Next, you tell the robot to turn around 180 degrees, then drive forward again. If you drew this motion on a map, the robot has just driven out to a point (+x) and gone back to the start (-x), but from the robot's prespective, it only moved forward (+x), turned, and moved forward again. This is very relevant to us, as we control the robot's motion only through z-axis rotations and x-axis translations. Frames of reference tell us how to figure out motion on the world map from the robot's movements and rotations relative to its current position, and vice versa. In other words, frames of reference are used to track the difference between the origin points and axes of two planes or objects (In this case the world map and the robot).

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

*Transform*
The Transform class takes types that don't necessarily match the types we use in the rest of the node. Make sure you give your input in the right format (it may help to declare these objects in the 'Transform()' declaration itself)

## 12. The Reset Service

Add a service of type `std_srvs/Trigger` that zeroes your position/heading estimate back to `(0, 0, 0)`. This lets other ROS nodes request that the odometry node zero itself. 

We're not giving you starter code for this. Go look at the slides, or better yet, the ROS2 docs or an online tutorial on services, to see if you can figure it out for yourself. Working together is also encouraged.

Some things to keep in mind while researching and referencing the slides:
- Where does the 'std_srvs/Trigger' type need to be imported from?
- The 'std_srvs' objects all follow the same format. How does this format apply to the 'Trigger' type compared to the other 'std_srvs' types?
- What types do service callbacks take? What type do they it return? 

To test this, the easiest thing to do is to reset, drive the square (§13), and confirm it starts from the same place every time. It'll also make general testing easier. You'll also need to find the right command to run the service.

A common error to watch out for: Zeroing doesn't automatically publish the new state, so you need to make sure the *TF broadcast* reflects the reset immediately, not just the next publisher call, so downstream visualization doesn't briefly show an outdated pose.



## 13. Running With Encoder Simulation

To run the simplified nav stack, you need to launch three separate packages. Launching `core.launch.py` brings up (starts) the core robot functionality, `localization.launch.py` starts the node you just made, and `simulation.launch.py` starts the demo encoder simulation that publishes to the 'enc_vel/raw' topic:

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

## 14. Running With the Rosbag

`bags/` contains a recorded rosbag of a heart-shaped trajectory on Maverick (the actual robot, not the simulation, so it also has noise). Launch the robot stack like you did in (§13), but instead of launching the simulation package, use the following ROS command to replay the previously recorded ROS bag.

```
ros2 bag play bags/<heart-bag-name>
```

Replaying ROS bags is a valuable tool for us, because it allows us to test our algorithms and visualization tools against known sets of input that either replay or closely replicate actual robot conditions. If we know that the ROS bag represents driving in a heart, and your node doesn't show that, we can tell it needs to be debugged or tuned.

## 15. Visualizing in RViz2

RViz is a useful tool for quickly visualizing different localization-related data topics in Maverick's stack. Rather than echoing a topic and reading the output, RViz lets us see things like CV output, intended path, and for our purposes, odometry in a simulated space.
To launch RViz, open a new terminal (do you have enough yet?) and run the following command:

```
rviz2
```
If you have a certain set of topics you regularly want to look at togehter, you can save them as a custom configuration. For this project, use RViz's **File → Open Config** and select `src/bringup/rviz/onboarding.rviz` in the onboarding repo, which sets the fixed frame to `odom` and has RobotModel, TF, and Odometry displays already added.


## 16. Formatting & Linting
We also have some formatting commands to make sure your packages match our preferred organization style:

```
just format   # auto-formats all source files
just lint     # runs the same checks CI (Continuous Integration) runs
```
*Run both before proceeding.* 

## 17. Opening Your PR

Commit your changes, and push to and publish your branch. Open it in Github, and open a Pull Request against `main` to submit your changes. The PR template will ask for:

- A screenshot of RViz showing a valid trajectory against the mock publisher (square)
- A screenshot of RViz showing a valid trajectory against the rosbag (heart)

CI (Continuous Integration checks) must pass (build + test + lint) before your PR is considered complete.

If it's approved, congratulations! You're done with onboarding. If not, see if you can find the error, and if you can't ask the leads.
Whatever you do, **do not merge your pull request to main.** Keep main clean for everyone else!
If you have any further questions about Git, ask the leads, your peers, or look it up yourself.

https://www.kern-it.be/en/definitions/pull-request/
https://docs.github.com/en/pull-requests/reference/pull-requests