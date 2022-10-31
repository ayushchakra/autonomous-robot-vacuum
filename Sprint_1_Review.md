#PIE Sprint 1 Review

##Lily, KD, Ayush, Jo, Omar
##31 October 2021


Our PIE project’s overarching goal is to make an autonomous, omnidirectional robot
that vacuums the floor it is traversing. The autonomy is achieved with a 360 degree
LiDAR scanner and a Raspberry Pi to control the robot’s direction based on scanner
readings. The robot uses mecanum wheels, which allow for omnidirectional travel to
easily avoid obstacles.

The goal for Sprint 1 was to complete our MVP, which was a basic robot with mecanum
wheels, basic DC motors, an Arduino, and keyboard controls. This MVP was to prove that
the robot could move in multiple directions, and the direction could be changed by
some kind of input.

Besides the overall goal of the MVP, each member had their own individual goal. Lily’s
goal was to gain experience with electronics in robotics, and learn more about stepper
motors/the infrastructure required to support them. KD’s goal was to get more
experience with research and relaying information with the team. Omar’s goal was to
gain experience with embedded systems and learn how to work on a multidisciplinary
team. Jo’s goal was ??. Ayush’s goal was ???.

Currently, the project has just passed over the MVP stage. A custom chassis lets 4
motors and an Arduino attach directly onto it. The Arduino is wired to 4 DC motors,
which each power a mecanum wheel. The Arduino script allows for movement in 8
directions: forward, backward, left, right, and the four diagonals between.
Additionally, this script takes keyboard inputs from the user to change the robot’s
direction.

From all of our research, we have determined that the biggest technical risk is the
feasibility of implementing a vacuuming feature. We are having difficulty figuring out
how to make a cheap, compact system so it fits both within our budget and on the
chassis. Additionally, we are struggling to figure out if we want the vacuum to be
omnidirectional as well. It would make sense to be able to vacuum in any direction that
the robot moves in, yet having multiple holes to vacuum from is difficult. To address
this, we have concluded as a team that we are not 100% committed to the vacuuming idea,
and we are open to ideate on other services that our robot could execute.

After completing Sprint 1 and the MVP, the features we want to implement in the final
product have not yet changed. However, for the vacuuming feature, we have all become
more unsure of whether we want to include it or not. More consideration will have to
occur for us to come to a final decision on that.

Sprint 2 will iterate upon the current MVP design to make the robot autonomously
navigate obstacles, rather than use manual keyboard inputs. To our current design,
we will mount a LiDAR, replace the Arduino with a Raspberry Pi, add code to recognize
obstacles from LiDAR readings, and change the current code to change the robot’s
direction as needed. Another change will be in replacing the DC motors with stepper
motors for more precise control and greater torque. The circuit will include stepper
motor controller boards, and the code for directional control will be adapted to work
with the stepper motors.
