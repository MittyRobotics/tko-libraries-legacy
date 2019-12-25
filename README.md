
# tko-libraries
Main code libraries for FRC Team 1351, TKO.
 
## Module Structure
 - ### ```data-collection```
	 - Contains functions for collecting and managing data, such as the TimeMonitor, which can measure execution time of specific code
 -  ### ```datatypes```
	 - Contains all of the standardized datatypes, math, and classes for all libraries
	 - Contains positioning, geometry, motion, and more
 -  ### ```motion-profiling```
	 - Contains motion profiling and velocity controlling features, such as the TrapezoidalMotionProfile and SafeVelocityController
 -  ### ```path-following```
	 - Contains the autonomous path following code
	 - Has different path following controllers, such as the PurePursuitController and the RamseteController
	 - Holds main path following class, PathFollower
 -  ### ```path-generation```
	 - Contains the path generation code
	 - Holds the different path generation algorithms for ```path-following``` as well as all path generation related code
 -  ### ```simulation```
	 - Contains robot simulator and system modeling code
	 - Used for simulating and visualizing chassis or system movement, and can be used to test motion code such as ```path-following``` or ```motion-profiling``` on a simulated robot.
 -  ### ```vision```
	 - Contains vision related code
	 - Holds the Limelight class which handles our interface with the Limelight Vision Camera
 -  ### ```visualization```
	 - Contains graphing and visualization utilities
	 - Holds a universal Graph class that can be used to graph data as well as specific graphs, such as RobotGraph which can graph a representation of a chassis from the top down
	 - Contains different, customizable graph themes, which we all know is the most important part!
