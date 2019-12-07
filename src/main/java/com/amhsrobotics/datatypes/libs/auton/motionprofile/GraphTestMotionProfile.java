package com.amhsrobotics.datatypes.libs.auton.motionprofile;

import com.amhsrobotics.datatypes.libs.datatypes.MechanismBounds;
import com.amhsrobotics.datatypes.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.datatypes.libs.visualization.graphs.GraphMotionProfile;

/**
 * Testing motion profile class. This generates a test motion profile and graphs it.
 */
public class GraphTestMotionProfile {
    public static void main(String... args){
        double acceleration = 20; 		//units/sec^2
        double deceleration = 13; 		//units/sec^2
        double maxVelocity = 18.844194345562954; 		//units/sec
        double startVelocity = 18.844194345562954; 		//units/sec
        double endVelocity = 10.0; 		//units/sec
        double lowerPositionBound = 0; 		//units
        double upperPositionBound = 0; 	//units
        double currentPosition = 0; 		//units
        double setpoint =  7.821977362208773; 			//units

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
