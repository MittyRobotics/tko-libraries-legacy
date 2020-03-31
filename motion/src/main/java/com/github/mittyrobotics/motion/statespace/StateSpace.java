package com.github.mittyrobotics.motion.statespace;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.motion.OverrideMethod;
import com.github.mittyrobotics.motion.SCurveMotionProfile;
import com.github.mittyrobotics.visualization.MotorGraph;
import org.ejml.simple.SimpleMatrix;

public class StateSpace {
    SimpleMatrix r, A, B, C, D, K, Kff, xHat, x, y, u, kalmanGain;
    double uMin, uMax;

    public static void main(String[] args) {
        StateSpace stateSpace = new StateSpace();
        stateSpace.r = new SimpleMatrix(new double[][]{
                {0},
                {0}
        });
        stateSpace.A = new SimpleMatrix(new double[][]{
                {1, 0.00343237},
                {0, 0.43637396}
        });
        stateSpace.B = new SimpleMatrix(new double[][]{
                {0.00021138},
                {0.07364964}
        });
        stateSpace.C = new SimpleMatrix(new double[][]{
                {1, 0}
        });
        stateSpace.D = new SimpleMatrix(new double[][]{
                {0}
        });
        stateSpace.u = new SimpleMatrix(new double[][]{
                {0}
        });
        stateSpace.x = new SimpleMatrix(new double[][]{
                {0},
                {0}
        });
        stateSpace.K = new SimpleMatrix(new double[][]{
                {232.76812611, 5.54069388}
        });
        stateSpace.Kff = new SimpleMatrix(new double[][]{
                {0.03896855, 13.57768762}
        });
        stateSpace.xHat = new SimpleMatrix(new double[][]{
                {0},
                {0}
        });
        stateSpace.kalmanGain = new SimpleMatrix(new double[][]{
                {0.99999602},
                {0.73475794}
        });
        stateSpace.uMin = -12;
        stateSpace.uMax = 12;

        SCurveMotionProfile motionProfile = new SCurveMotionProfile(new MotionState(0,0,0), new MotionState(5,0,0), 1,
                1, 2,
                1,
                OverrideMethod.OVERSHOOT);
        MotorGraph graph = new MotorGraph();
        for (double t = 0; t < 20; t += 0.00505) {
            double setpoint = motionProfile.calculateState(t).getPosition();
            stateSpace.update(new SimpleMatrix(new double[][]{
                    {setpoint},
                    {0}
            }));
            graph.addVoltage(stateSpace.u.get(0), t);
            graph.addPosition(stateSpace.x.get(0), t);
            graph.addVelocity(stateSpace.x.get(1), t);
            graph.addSetpoint(setpoint, t);
            graph.addAcceleration(motionProfile.calculateState(t).getVelocity(), t);
        }
    }

    public void update(SimpleMatrix nextR) {
        updatePlant();
        correctObserver();
        updateController(nextR);
        predictObserver();
    }

    public void updatePlant() {
        x = A.mult(x).plus(B.mult(u));
        y = C.mult(x).plus(D.mult(u));
    }

    public void correctObserver() {
        xHat = xHat.plus(kalmanGain.mult(y.minus(C.mult(xHat).minus(D.mult(u)))));
    }

    public void updateController(SimpleMatrix nextR) {
        SimpleMatrix _u = K.mult(nextR.minus(xHat));
        SimpleMatrix uff = Kff.mult(nextR.minus(A.mult(r)));
        u = clip(_u.plus(uff), uMin, uMax);
    }

    public void predictObserver() {
        xHat = A.mult(xHat).plus(B.mult(u));
    }

    public SimpleMatrix clip(SimpleMatrix a, double min, double max) {
        for (int row = 0; row < a.numRows(); row++) {
            for (int col = 0; col < a.numCols(); col++) {
                a.set(row, col, Math.max(min, Math.min(max, a.get(row, col))));
            }
        }
        return a;
    }
}
