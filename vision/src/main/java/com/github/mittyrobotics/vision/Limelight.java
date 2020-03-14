/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.vision;

import com.github.mittyrobotics.vision.enums.LimelightCameraMode;
import com.github.mittyrobotics.vision.enums.LimelightLEDMode;
import com.github.mittyrobotics.vision.enums.LimelightSnapshotMode;
import com.github.mittyrobotics.vision.enums.LimelightStreamMode;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Singleton that handles all Limelight related tasks and functions.
 */
public class Limelight {
    private static Limelight ourInstance = new Limelight();
    private boolean hasValidTarget;
    private double yawToTarget;
    private double pitchToTarget;
    private double targetArea;
    private double targetScreenRotation;
    private double limelightLatency;
    private double boxShortestSide;
    private double boxLongestSide;
    private double boxHorizontalSide;
    private double boxVerticalSide;
    private double[] target3DCamera;
    private double[] targetCornerX;
    private double[] targetCornerY;
    private double DEFAULT_VALUE = -1000;

    private Limelight() {

    }

    public static Limelight getInstance() {
        return ourInstance;
    }

    public void initDefaultLimelightSettings() {
        setPipeline(0);
        setLedMode(LimelightLEDMode.On);
        setCameraMode(LimelightCameraMode.Vision);
        setStreamMode(LimelightStreamMode.Secondary);
        setSnapshotMode(LimelightSnapshotMode.Off);
    }

    /**
     * Reads the Limelight's values from NetworkTables and does necessary calculations.
     */
    public void updateLimelightValues() {
        this.hasValidTarget =
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(DEFAULT_VALUE) == 1;
        if (hasValidTarget) {
            this.yawToTarget =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(DEFAULT_VALUE);
            this.pitchToTarget =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(DEFAULT_VALUE);
            this.targetArea =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(DEFAULT_VALUE);
            this.targetScreenRotation =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(DEFAULT_VALUE);
            this.limelightLatency =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(DEFAULT_VALUE);
            this.boxShortestSide =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(DEFAULT_VALUE);
            this.boxLongestSide =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(DEFAULT_VALUE);
            this.boxHorizontalSide =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(DEFAULT_VALUE);
            this.boxVerticalSide =
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(DEFAULT_VALUE);
            this.target3DCamera = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(
                    new double[]{DEFAULT_VALUE, DEFAULT_VALUE, DEFAULT_VALUE, DEFAULT_VALUE, DEFAULT_VALUE,
                            DEFAULT_VALUE});
            this.targetCornerX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornx")
                    .getDoubleArray(new double[]{DEFAULT_VALUE});
            this.targetCornerY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcorny")
                    .getDoubleArray(new double[]{DEFAULT_VALUE});
        }
    }

    public LimelightLEDMode getLedMode() {
        return LimelightLEDMode.valueOf(
                (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(-1000));
    }

    /**
     * Sets the LED mode on the Limelight camera.
     * <p>
     * 0	use the LED Mode set in the current pipeline 1	force off 2	force blink 3	force on
     *
     * @param ledMode enum containing the different modes for leds.
     */
    public void setLedMode(LimelightLEDMode ledMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode")
                .setNumber(ledMode.value); //	Sets limelight’s LED state
    }

    public LimelightCameraMode getCameraMode() {
        return LimelightCameraMode.valueOf(
                (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getNumber(-1000));
    }

    /**
     * Sets the camera mode on the Limelight camera.
     * <p>
     * 0	Vision processor 1	Driver Camera (Increases exposure, disables vision processing) pipeline	Sets limelight’s
     * current pipeline
     *
     * @param cameraMode enum containing the different modes of camera
     */
    public void setCameraMode(LimelightCameraMode cameraMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode")
                .setNumber(cameraMode.value); //	Sets limelight’s operation mode
    }

    public double getPipeline() {
        return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getNumber(-1000);
    }

    /**
     * Sets the active pipeline on the Limelight camera. These pipelines are configured in the limelight configuration
     * tool and range from 0-9.
     *
     * @param pipelineID ID of new active pipeline
     */
    public void setPipeline(int pipelineID) {
        if (pipelineID > 9 || pipelineID < 0) {
            pipelineID = 0;
        }
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline")
                .setNumber(pipelineID); //	Sets limelight’s operation mode
    }

    public LimelightStreamMode getStreamMode() {
        return LimelightStreamMode.valueOf(
                (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").getNumber(-1000));
    }

    /**
     * Sets the stream configuration on the Limelight camera.
     * <p>
     * 0	Standard - Side-by-side streams if a webcam is attached to Limelight 1	PiP Main - The secondary camera stream
     * is placed in the lower-right corner of the primary camera stream 2	PiP Secondary - The primary camera stream is
     * placed in the lower-right corner of the secondary camera stream
     *
     * @param streamMode enum containing the different streaming configurations
     */
    public void setStreamMode(LimelightStreamMode streamMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream")
                .setNumber(streamMode.value); //Sets limelight’s streaming mode
    }

    public LimelightSnapshotMode getSnapshotMode() {
        return LimelightSnapshotMode.valueOf(
                (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").getNumber(-1000));
    }

    /**
     * Sets the snapshot mode on the Limelight camera.
     * <p>
     * 0	Stop taking snapshots 1	Take two snapshots per second
     *
     * @param snapshotMode enum containing the different snapshot modes
     */
    public void setSnapshotMode(LimelightSnapshotMode snapshotMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot")
                .setNumber(snapshotMode.value); //Allows users to take snapshots during a match
    }

    /**
     * Puts the Limelight into vision mode, with the camera settings turned to vision mode (low exposure) and the LEDs
     * on.
     * <p>
     * Vision mode sets the camera properties to allow for optimum vision tracking and turns the LEDs on so the target
     * can be tracked.
     */
    public void enableVisionMode() {
        setCameraMode(LimelightCameraMode.Vision);
        setLedMode(LimelightLEDMode.On);
    }

    /**
     * Puts the Limelight into driver mode, with the camera settings turned to driver mode (regular exposure) and the
     * LEDs off.
     * <p>
     * Driver mode allows the driver to see through the Limelight camera and turns off the LEDs so it does not distract
     * others on the field.
     */
    public void enableDriverMode() {
        setCameraMode(LimelightCameraMode.Driver);
        setLedMode(LimelightLEDMode.Off);
    }

    public boolean isHasValidTarget() {
        return hasValidTarget;
    }

    public double getYawToTarget() {
        if (yawToTarget == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getYawToTarget() could not be could not be received!");
            return 0;
        }
        return yawToTarget;
    }

    public double getPitchToTarget() {
        if (pitchToTarget == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getPitchToTarget() could not be could not be received!");
            return 0;
        }
        return pitchToTarget;
    }

    public double getTargetArea() {
        if (targetArea == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getTargetArea() could not be could not be received!");
            return 0;
        }
        return targetArea;
    }

    public double getTargetScreenRotation() {
        if (targetScreenRotation == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getTargetScreenRotation() could not be could not be received!");
            return 0;
        }
        return targetScreenRotation;
    }

    public double getLimelightLatency() {
        if (limelightLatency == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getLimelightLatency() could not be could not be received!");
            return 0;
        }
        return limelightLatency;
    }

    public double getBoxShortestSide() {
        if (boxShortestSide == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getBoxShortestSide() could not be could not be received!");
            return 0;
        }
        return boxShortestSide;
    }

    public double getBoxLongestSide() {
        if (boxLongestSide == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getBoxLongestSide() could not be could not be received!");
            return 0;
        }
        return boxLongestSide;
    }

    public double getBoxHorizontalSide() {
        if (boxHorizontalSide == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getBoxHorizontalSide() could not be could not be received!");
            return 0;
        }
        return boxHorizontalSide;
    }

    public double getBoxVerticalSide() {
        if (boxVerticalSide == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getBoxVerticalSide() could not be could not be received!");
            return 0;
        }
        return boxVerticalSide;
    }

    public double get3DCameraX() {
        if (target3DCamera[0] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCameraX() could not be could not be received!");
            return 0;
        }
        return target3DCamera[0];
    }

    public double get3DCameraY() {
        if (target3DCamera[1] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCameraY() could not be could not be received!");
            return 0;
        }
        return target3DCamera[1];
    }

    public double get3DCameraZ() {
        if (target3DCamera[2] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCameraZ() could not be could not be received!");
            return 0;
        }
        return target3DCamera[2];
    }

    public double get3DCameraPitch() {
        if (target3DCamera[3] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCameraPitch() could not be could not be received!");
            return 0;
        }
        return target3DCamera[3];
    }

    public double get3DCameraYaw() {
        if (target3DCamera[4] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCameraYaw() could not be could not be received!");
            return 0;
        }
        return target3DCamera[4];
    }

    public double get3DCameraRoll() {
        if (target3DCamera[5] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCameraRoll() could not be could not be received!");
            return 0;
        }
        return target3DCamera[5];
    }

    public double[] get3DCamera() {
        if (target3DCamera.length != 0 && target3DCamera[0] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight get3DCamera() could not be could not be received!");
            return new double[]{0, 0, 0, 0, 0, 0};
        }
        return target3DCamera;
    }

    public double[] getTargetCornerX() {
        if (targetCornerX.length != 0 && targetCornerX[0] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getTargetCornerX() could not be could not be received!");
            return new double[]{0};
        }
        return targetCornerX;
    }

    public double[] getTargetCornerY() {
        if (targetCornerY.length != 0 && targetCornerY[0] == DEFAULT_VALUE) {
            System.out.println("WARNING: Limelight getTargetCornerY() could not be could not be received!");
            return new double[]{0};
        }
        return targetCornerY;
    }
}
