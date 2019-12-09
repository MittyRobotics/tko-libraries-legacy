package com.github.mittyrobotics.libs.auton.path.follow;

import com.github.mittyrobotics.libs.util.geometry.Arc;
import com.github.mittyrobotics.libs.util.geometry.Position;
import com.github.mittyrobotics.libs.util.geometry.Transform;
import com.github.mittyrobotics.libs.auton.path.generation.Path;
import com.github.mittyrobotics.libs.datatypes.DrivetrainVelocities;
import com.github.mittyrobotics.libs.util.path.PathSegment;

public class AutonDriver {
    private static AutonDriver instance = new AutonDriver();
    private Path path;

    public static AutonDriver getInstance() {
        return instance;
    }

    private AutonDriver(){

    }

    public void initPurePursuit(Path path){

        this.path = path;
    }

    public DrivetrainVelocities updatePurePursuit(double t, Transform robotTransform, double lookaheadDistance){
        //get closest segment
        PathSegment closestSegment = path.getClosestSegment(robotTransform, Path.RoundMode.ROUND_DOWN,0);

        //get target segment
        PathSegment targetSegment = null;

        //get closest point to robot transform in target segment
        Position closestPoint = closestSegment.getParallelIntersection(robotTransform);

        //get closest point to target circle in target segment
        Position targetPoint = targetSegment.getIntersectionPointWithCircle(new Arc(closestPoint,lookaheadDistance));

        //get distance of closest point on segment along segment
        double positionOnSegment = closestPoint.distance(closestSegment.getStartPoint().getPosition());

        //get the velocity from the motion profile of the closest segment using motionProfile.getTimeAtPosition();
        double robotVelocity = closestSegment.getMotionProfile().getTimeAtPosition(positionOnSegment);

        Position robotCentricTargetPoint = new Transform(targetPoint).relativeTo(robotTransform).getPosition();

        return PurePursuitController.getInstance().purePursuitController(robotCentricTargetPoint,robotVelocity);
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path path) {
        this.path = path;
    }
}
