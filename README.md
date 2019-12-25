
# tko-libraries
Main code libraries for FRC Team 1351, TKO.
 
## Module Structure
 - ### [```data-collection```](https://github.com/MittyRobotics/tko-libraries/tree/master/data-collection/src/main/java/com/github/mittyrobotics/datacollection)
	 - Contains functions for collecting and managing data, such as the [TimeMonitor](https://github.com/MittyRobotics/tko-libraries), which can measure execution time of specific code
 -  ### [```datatypes```](https://github.com/MittyRobotics/tko-libraries/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes)
	 - Contains all of the standardized datatypes, math, and classes for all libraries
	 - Contains [positioning](https://github.com/MittyRobotics/tko-libraries/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/positioning), [geometry](https://github.com/MittyRobotics/tko-libraries/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/geometry), [motion](https://github.com/MittyRobotics/tko-libraries/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/motion), and more
 -  ### [```motion-profiling```](https://github.com/MittyRobotics/tko-libraries/tree/master/motion-profiling/src/main/java/com/github/mittyrobotics/motionprofile)
	 - Contains motion profile generation and velocity controlling features, such as the [TrapezoidalMotionProfile](https://github.com/MittyRobotics/tko-libraries/blob/master/motion-profiling/src/main/java/com/github/mittyrobotics/motionprofile/TrapezoidalMotionProfile.java) and [SafeVelocityController](https://github.com/MittyRobotics/tko-libraries/blob/master/motion-profiling/src/main/java/com/github/mittyrobotics/motionprofile/SafeVelocityController.java)
 -  ### [```path-following```](https://github.com/MittyRobotics/tko-libraries/tree/master/path-following/src/main/java/com/github/mittyrobotics/path/following)
	 - Contains the autonomous path following code
	 - Has different path following controllers, such as the [PurePursuitController](https://github.com/MittyRobotics/tko-libraries/blob/master/path-following/src/main/java/com/github/mittyrobotics/path/following/controllers/PurePursuitController.java) and the [RamseteController](https://github.com/MittyRobotics/tko-libraries/blob/master/path-following/src/main/java/com/github/mittyrobotics/path/following/controllers/RamseteController.java)
	 - Holds main path following class, [PathFollower](https://github.com/MittyRobotics/tko-libraries/blob/master/path-following/src/main/java/com/github/mittyrobotics/path/following/PathFollower.java)
 -  ### [```path-generation```](https://github.com/MittyRobotics/tko-libraries/tree/master/path-generation/src/main/java/com/github/mittyrobotics/path/generation)
	 - Contains the path generation code
	 - Holds the different path generation algorithms for [```path-following```](https://github.com/MittyRobotics/tko-libraries/tree/master/path-following/src/main/java/com/github/mittyrobotics/path/following) as well as all path generation related code
 -  ### [```simulation```](https://github.com/MittyRobotics/tko-libraries/tree/master/simulation/src/main/java/com/github/mittyrobotics/simulation)
	 - Contains robot simulator and system modeling code
	 - Used for simulating and visualizing chassis or system movement, and can be used to test motion code such as [```path-following```](https://github.com/MittyRobotics/tko-libraries/tree/master/path-following/src/main/java/com/github/mittyrobotics/path/following) or [```motion-profiling```](https://github.com/MittyRobotics/tko-libraries/tree/master/motion-profiling/src/main/java/com/github/mittyrobotics/motionprofile) on a simulated robot.
 -  ### [```vision```](https://github.com/MittyRobotics/tko-libraries/tree/master/vision/src/main/java/com/github/mittyrobotics/vision)
	 - Contains vision related code
	 - Holds the [Limelight](https://github.com/MittyRobotics/tko-libraries/blob/master/vision/src/main/java/com/github/mittyrobotics/vision/Limelight.java) class which handles our interface with the [Limelight Vision Camera](https://limelightvision.io/)
 -  ### [```visualization```](https://github.com/MittyRobotics/tko-libraries/tree/master/visualization/src/main/java/com/github/mittyrobotics/visualization)
	 - Contains graphing and visualization utilities
	 - Holds a universal [Graph](https://github.com/MittyRobotics/tko-libraries/blob/master/visualization/src/main/java/com/github/mittyrobotics/visualization/graphs/Graph.java) class that can be used to graph data as well as specific graphs, such as RobotGraph which can graph a representation of a chassis from the top down
	 - Contains different, customizable [GraphThemes](https://github.com/MittyRobotics/tko-libraries/tree/master/visualization/src/main/java/com/github/mittyrobotics/visualization/themes), which we all know is the most important part!
