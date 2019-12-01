package com.amhsrobotics.libs.auton.motionprofile;

import com.amhsrobotics.libs.datatypes.MechanismBounds;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.visualization.graphs.GraphMotionProfile;

/**
 * Testing motion profile class. This generates a test motion profile and graphs it.
 */
public class GraphTestMotionProfile {
    public static void main(String... args){
        double acceleration = 5; 		//units/sec^2
        double deceleration = 5; 		//units/sec^2
        double maxVelocity = 20.0; 		//units/sec
        double startVelocity = 0; 		//units/sec
        double endVelocity = 0; 		//units/sec
        double lowerPositionBound = 0; 		//units
        double upperPositionBound = 0; 	//units
        double currentPosition = 0; 		//units
        double setpoint = 10; 			//units

        VelocityConstraints velocityConstraints = new VelocityConstraints(
                acceleration,
                deceleration,
                maxVelocity,
                startVelocity,
                endVelocity
        );

        MechanismBounds mechanismBounds = new MechanismBounds(
                currentPosition,
                lowerPositionBound,
                upperPositionBound
        );

        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(
                setpoint,
                velocityConstraints,
                mechanismBounds
        );

        new GraphMotionProfile(motionProfile);

    }
}
