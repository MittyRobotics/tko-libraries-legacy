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
    Contains [positioning](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/positioning)
    , [geometry](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/datatypes/src/main/java/com/github/mittyrobotics/datatypes/geometry)
    , [motion](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion)
    , and more
-  ### [```motion-profiling```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/profiles)
    - Contains motion profile generation and velocity controlling features, such as
      the [DynamicTrapezoidalMotionProfile](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/profiles/DynamicTrapezoidalMotionProfile.java)
      and [SCurveMotionProfile](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/motion/src/main/java/com/github/mittyrobotics/motion/profiles/SCurveMotionProfile.java)
-  ### [```path-following```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/motion/src/main/java/com/github/mittyrobotics/motion/pathfollowing)
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
-  ### [```visualization```](https://github.com/MittyRobotics/tko-libraries-legacy/tree/master/visualization/src/main/java/com/github/mittyrobotics/visualization)
    - Contains graphing and visualization utilities
    - Holds a
      universal [Graph](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/visualization/src/main/java/com/github/mittyrobotics/visualization/Graph.java)
      class that can be used to graph data as well as specific graphs, such as RobotGraph which can graph a
      representation of a chassis from the top down
    - Contains different,
      customizable [GraphThemes](https://github.com/MittyRobotics/tko-libraries-legacy/blob/master/visualization/src/main/java/com/github/mittyrobotics/visualization/themes/GraphTheme.java).
