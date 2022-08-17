# tko-libraries

Main code libraries for FRC Team 1351, TKO.

## Module Structure

- ### [```data-collection```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/data-collection/src/main/java/com/github/mittyrobotics/datacollection)
    - Contains functions for collecting and managing data, such as
      the [TimeMonitor](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/data-collection/src/main/java/com/github/mittyrobotics/datacollection/performance/TimeMonitor.java), which can measure execution time of specific
      code
-  ### [```datatypes```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes)
    - Contains all of the standardized datatypes, math, and classes for all libraries
    -
    Contains [positioning](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/positioning), [geometry](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/geometry), [motion](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/motion),
   and more
-  ### [```motion```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion)
    - Contains motion control code, from [path followers](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/pathfollowing) to [models](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/modeling/models) and [controllers](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/controllers).
    - Contains motion profile generation and velocity controlling features, such as
      the [DynamicTrapezoidalMotionProfile](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/profiles/DynamicTrapezoidalMotionProfile.java)
      and [SCurveMotionProfile](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/profiles/SCurveMotionProfile.java)
    -  #### [Modeling](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/modeling)
        - Contains code for state-space control and model simulations. Has pre-derived models for common mechanisms such as [flywheels](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/modeling/models/FlywheelModel.java), [arms](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/modeling/models/SingleJointedArmModel.java), and [drivetrains](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/modeling/models/DrivetrainModel.java).
        - Contains the main [Plant](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/modeling/Plant.java) class for model-based control.
    -  #### [Autonomous Navigation](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/pathfollowing)
        - Contains the autonomous path following code
        - Has different path following controllers, such as
      the [PurePursuitController](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/controllers/PurePursuitController.java)
      and
      the [RamseteController](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/controllers/RamseteController.java)
        - Holds main path following
      class, [PathFollower](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/pathfollowing/PathFollower.java)
-  ### [```path-generation```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/path-generation/src/main/java/com/github/mittyrobotics/path/generation)
    - Contains the Path generation code and the
      main [Path](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/path-generation/src/main/java/com/github/mittyrobotics/path/generation/Path.java)
      class
    - Holds the different path generation algorithms
      for  [```path-following```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/pathfollowing)
      as well as all path generation related code
    - Currently the main path generation algorithm is
      the [QuinticHermiteSpline](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/path-generation/src/main/java/com/github/mittyrobotics/path/generation/splines/QuinticHermiteSpline.java)
-  ### [```simulation```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/simulation/src/main/java/com/github/mittyrobotics/simulation)
    - Contains robot simulator and system modeling code
    - Used for simulating and visualizing chassis or system movement, and can be used to test motion code such
      as  [```path-following```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/pathfollowing)
      or [```motion-profiling```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/profiles)
      on a simulated robot.
    - Example Simulation: 
        ```java        
        DrivetrainModel drivetrainModel =
                new DrivetrainModel(50 * Conversions.LBS_TO_KG, 1.585, 20 * Conversions.IN_TO_M,
                        30 * Conversions.IN_TO_M, new CIMMotor(2), 7.0, 2 * Conversions.IN_TO_M);
        PathFollowerSimRobot robot = new PathFollowerSimRobot(new SimDrivetrain(drivetrainModel));

        RobotSimulator simulator = new RobotSimulator(robot, 0.02, new RobotGraph());
-  ### [```visualization```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/visualization/src/main/java/com/github/mittyrobotics/visualization)
    - Contains graphing and visualization utilities
    - Holds a
      universal [Graph](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/visualization/src/main/java/com/github/mittyrobotics/visualization/Graph.java)
      class that can be used to graph data as well as specific graphs, such as RobotGraph which can graph a
      representation of a chassis from the top down
    - Contains different,
      customizable [GraphThemes](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/visualization/src/main/java/com/github/mittyrobotics/visualization/themes/GraphTheme.java).
