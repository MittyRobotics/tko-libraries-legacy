package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.motion.observers.KalmanFilter;
import com.github.mittyrobotics.visualization.Graph;
import org.ejml.simple.SimpleMatrix;
import org.jfree.data.xy.XYDataItem;

public class TestKalmanFilter {
    public static void main(String[] args) {
        double dt = 0.01;
        SimpleMatrix A = new SimpleMatrix(new double[][]{{0}});
        SimpleMatrix B = new SimpleMatrix(new double[][]{{1}});
        SimpleMatrix H = new SimpleMatrix(new double[][]{{1}});
        SimpleMatrix Q = new SimpleMatrix(new double[][]{{5}});
        SimpleMatrix R = new SimpleMatrix(new double[][]{{20}});
        KalmanFilter filter = new KalmanFilter(A, B, H, Q, R);
        Graph graph = new Graph();

        double totalTime = 1;
        double oldY = 0;
        double oldYVel = 0;
        for(double t = 0; t < totalTime; t += dt){
            double y = t*t;
            double yVel = (y-oldY)/dt;
            double yAcc = (yVel-oldYVel)/dt;

            double measurementNoise = 0.05;
            double noise = (Math.random()*2-1)*measurementNoise;
            double noise1 = (Math.random()*2-1)*measurementNoise;

            graph.addToSeries("Actual Pos", new XYDataItem(t, y));
//            graph.addToSeries("Actual Vel", new XYDataItem(t, yVel));

            SimpleMatrix u = new SimpleMatrix(new double[][]{{y*.6}});

            filter.predict(u);

            graph.addToSeries("Predict Pos", new XYDataItem(t, filter.getxHat().get(0)));
//            graph.addToSeries("Predict Vel", new XYDataItem(t, filter.getxHat().get(1)));


            SimpleMatrix z = new SimpleMatrix(new double[][]{{y+noise}});

            filter.correct(z);

            graph.addToSeries("Measurement Pos", new XYDataItem(t, z.get(0)));
//            graph.addToSeries("Measurement Vel", new XYDataItem(t, z.get(1)));


            graph.addToSeries("Correct Pos", new XYDataItem(t, filter.getxHat().get(0)));
//            graph.addToSeries("Correct Vel", new XYDataItem(t, filter.getxHat().get(1)));


            oldY = y;
            oldYVel = yVel;
            try {
                Thread.sleep((long) (totalTime/dt));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
