# MSc-Dissertation

### Best control policy for the setup
Use Cartesian impedance control tracking a curved 4-pass serpentine scan.
Because the phantom is:
    - 14 cm long and effectively straight in that direction
    - 12 cm wide and curved across that direction
    - height varies from 4.5 cm to 6 cm across the width

The cleanest policy is:
    - scan along the 14 cm straight direction
    - step across the 12 cm curved direction
    - change Z for each pass according to the curvature
    - use Cartesian impedance so contact is compliant on the real Panda

So the axes should stay:
    - scan_axis = 1 → along the 14 cm length
    - lateral_axis = 0 → across the 12 cm curved width
    - contact_axis = 2 → vertical/contact direction


### Geometry to use
Since the curved width is 12 cm, half-width is: 0.06 m
Since max height is 6 cm and min height is 4.5 cm, the height difference is: 0.015 m

A simple parabola is a very good first model:
```
z(x)=z_center−4.1667x^2
```
where:
    - x is lateral position in meters
    - z_center is the calibrated center contact height
    - at x = ±0.06 m, the surface is 0.015 m lower than the center

This matches our dimensions:
    - center highest
    - edges lower
    - straight along the 14 cm length


### Four scans to cover the whole area

Use 4 longitudinal passes across the width.
A good set of lateral line centers is:
```
x1 = -0.045 m
x2 = -0.015 m
x3 =  0.015 m
x4 =  0.045 m
```
These are good because they cover the 12 cm width without forcing the probe exactly onto the extreme edges.

For each pass, scan along:
```
y from -0.07 m to +0.07 m
```
because the total length is 14 cm.

Use a continuous serpentine pattern:
    - pass 1: y = -0.07 → +0.07
    - pass 2: y = +0.07 → -0.07
    - pass 3: y = -0.07 → +0.07
    - pass 4: y = +0.07 → -0.07

That gives continuous motion with no unnecessary lift-and-return.


### What changes with the curve

Because the curvature is across width, each longitudinal scan line has a different Z height.

Using the parabola:

- at x = ±0.045,
```
Δz=−4.1667⋅(0.045)^2≈−0.0084 m
```
so about 8.4 mm lower than center

- at x = ±0.015,
```
Δz=−4.1667⋅(0.015)2≈−0.00094 m
```
so about 0.94 mm lower than center

So our 4 scan heights relative to the center are approximately:
```
x = -0.045  → z = z_center - 8.4 mm
x = -0.015  → z = z_center - 0.9 mm
x =  0.015  → z = z_center - 0.9 mm
x =  0.045  → z = z_center - 8.4 mm
```
That is exactly the behavior wanted.


### Proper control architecture

For the real robot, the correct architecture is:

1. MoveIt
Use MoveIt only to:
move to a safe pre-scan pose
optionally move to the first scan start pose

2. Cartesian impedance controller
Use the Franka Cartesian impedance controller to track:
```
pd(t)=[xd(t),yd(t),zd(t)]
```

where:
x_d(t) is the current pass center or lateral transition
y_d(t) moves slowly along the 14 cm line
z_d(t) follows the curve

3. Fixed orientation
Keep the ultrasound probe orientation fixed initially.


### Continuous slow motion law
For each pass, let scan speed be slow and constant.
A good first real value is: 0.003 m/s. That is 3 mm/s.

With 14 cm length, one pass takes about:
```
0.14/0.003≈46.7 s
```

So 4 passes will take a bit over 3 minutes including small connectors. That is slow, controlled, and very suitable for ultrasound.
If that is too slow for development, start with: 0.005 m/s and reduce later.


### The exact reference path
Define:
y_min = -0.07
y_max = 0.07

lateral centers:
[-0.045, -0.015, 0.015, 0.045]

Then for pass i:
```
xd=xi
```
```
zd=zcenter−4.1667 xi2+zcontact
```​

and y_d moves linearly between y_min and y_max, alternating direction each pass.

That gives 4 full scans over the area.


### Connector motion between passes
To keep motion continuous, do not jump.
At the end of a pass:
keep y at the current end
move x slowly to the next lateral line
update z = z(x) continuously during that transition
So the connector also follows the curved surface.
That gives a truly continuous scan.

optical table height ≈ 5 cm
phantom height ≈ 4.5 to 6 cm
tool/end-effector height including scanner ≈ 22 cm

So the phantom top surface is roughly:
9.5 cm to 11 cm above the shared lab table
That is useful as a sanity check.

But for the robot controller, the important quantity is surface height in robot coordinates, not table coordinates.
So the right workflow is:
manually teach or estimate the center contact pose
use that as z_center
derive the other 3 pass heights from the parabola

#### Terminal 1
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch panda_lung_scan_noetic force_scan_hardware.launch
```

#### Terminal 2
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch panda_lung_scan_noetic moveit_to_start.launch
```

#### Terminal 3
```
rosservice call /controller_manager/switch_controller "{
  start_controllers: ['panda_ultrasound_force_scan_controller'],
  stop_controllers: ['position_joint_trajectory_controller'],
  strictness: 2,
  start_asap: false,
  timeout: 0.0
}"

rosservice call /controller_manager/switch_controller "{
  start_controllers: ['position_joint_trajectory_controller'],
  stop_controllers: ['panda_ultrasound_force_scan_controller'],
  strictness: 2,
  start_asap: false,
  timeout: 0.0
}"
```
