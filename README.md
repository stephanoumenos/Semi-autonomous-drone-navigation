# Semi-autonomous drone navigation
## CentraleSupélec Séquence Thématique 5


The goal of this project was building a semi-automonous drone platform that allows for preseting of several sophisticated behaviors.


To do so, we used the [ROS robotics middleware](https://en.wikipedia.org/wiki/Robot_Operating_System) to build on several techniques:

* Low level control: Mainly implementing PID controllers to transform desired linear and angular velocities to commands accepted by the drone (roll and pitch angles, vertical linear velocity as well as the angular velocity).
Also some other things that were needed, such as the joypad teleop and some manoeuvres.

* Behavior: ROS node that schedules determined behaviors for the drone to execute, such as "Hover", "Forward", "TurnLeft" etc.

* Visual Processing: Library developed to detect several vital visual detections for the drone's stability, such as the [Vanishing Point](https://en.wikipedia.org/wiki/Vanishing_point) detection and the floor and wall lines of the corridors.

More details can be found on [the subject's website](http://st5drone.metz.centralesupelec.fr/), along with a very nice illustrating video.

