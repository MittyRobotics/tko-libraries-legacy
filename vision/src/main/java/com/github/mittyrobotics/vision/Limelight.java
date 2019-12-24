/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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
	
	public static Limelight getInstance() {
		return ourInstance;
	}
	
	private Limelight() {
	
	}
	
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
	
	public void initDefaultLimelightSettings(){
		setPipeline(0);
		setLedMode(LimelightLEDMode.On);
		setCameraMode(LimelightCameraMode.Vision);
		setStreamMode(LimelightStreamMode.Secondary);
		setSnapshotMode(LimelightSnapshotMode.Off);
	}
	
	/**
	 * Reads the Limelight's values from NetworkTables and does necessary calculations.
	 */
	public void updateLimelightValues(){
		hasValidTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(-1000) == 1;
		yawToTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(-1000);
		pitchToTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(-1000);
		targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(-1000);
		targetScreenRotation = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(-1000);
		limelightLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(-1000);
		boxShortestSide = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(-1000);
		boxLongestSide = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(-1000);
		boxHorizontalSide = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(-1000);
		boxVerticalSide = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(-1000);
		target3DCamera = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{-1000,-1000,-1000,-1000,-1000,-1000});
		targetCornerX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornx").getDoubleArray(new double[]{-1000});
		targetCornerY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcorny").getDoubleArray(new double[]{-1000});
	}
	
	/**
	 * Sets the LED mode on the Limelight camera.
	 * <p>
	 * 0	use the LED Mode set in the current pipeline
	 * 1	force off
	 * 2	force blink
	 * 3	force on
	 *
	 * @param ledMode enum containing the different modes for leds.
	 */
	public void setLedMode(LimelightLEDMode ledMode) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode.value); //	Sets limelight’s LED state
	}
	
	/**
	 * Sets the camera mode on the Limelight camera.
	 * <p>
	 * 0	Vision processor
	 * 1	Driver Camera (Increases exposure, disables vision processing) pipeline	Sets limelight’s current pipeline
	 *
	 * @param cameraMode enum containing the different modes of camera
	 */
	public void setCameraMode(LimelightCameraMode cameraMode) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(cameraMode.value); //	Sets limelight’s operation mode
	}
	
	/**
	 * Sets the active pipeline on the Limelight camera. These pipelines are
	 * configured in the limelight configuration tool and range from 0-9.
	 *
	 * @param pipelineID ID of new active pipeline
	 */
	public void setPipeline(int pipelineID) {
		if (pipelineID > 9 || pipelineID < 0) {
			pipelineID = 0;
		}
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineID); //	Sets limelight’s operation mode
	}
	
	/**
	 * Sets the stream configuration on the Limelight camera.
	 * <p>
	 * 0	Standard - Side-by-side streams if a webcam is attached to Limelight
	 * 1	PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
	 * 2	PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
	 *
	 * @param streamMode enum containing the different streaming configurations
	 */
	public void setStreamMode(LimelightStreamMode streamMode) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(streamMode.value); //Sets limelight’s streaming mode
	}
	
	/**
	 * Sets the snapshot mode on the Limelight camera.
	 * <p>
	 * 0	Stop taking snapshots
	 * 1	Take two snapshots per second
	 *
	 * @param snapshotMode enum containing the different snapshot modes
	 */
	public void setSnapshotMode(LimelightSnapshotMode snapshotMode) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(snapshotMode.value); //Allows users to take snapshots during a match
	}
	
	public LimelightLEDMode getLedMode() {
		return LimelightLEDMode.valueOf((int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(-1000));
	}
	
	public LimelightCameraMode getCameraMode() {
		return LimelightCameraMode.valueOf((int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getNumber(-1000));
	}
	
	public double getPipeline() {
		return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getNumber(-1000);
	}
	
	public LimelightStreamMode getStreamMode() {
		return LimelightStreamMode.valueOf((int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").getNumber(-1000));
	}
	
	public LimelightSnapshotMode getSnapshotMode() {
		return LimelightSnapshotMode.valueOf((int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").getNumber(-1000));
	}
	
	/**
	 * Puts the Limelight into vision mode, with the camera settings turned to vision mode (low exposure)
	 * and the LEDs on.
	 * <p>
	 * Vision mode sets the camera properties to allow for optimum vision tracking and turns the LEDs on so the
	 * target can be tracked.
	 */
	public void enableVisionMode() {
		setCameraMode(LimelightCameraMode.Vision);
		setLedMode(LimelightLEDMode.On);
	}
	
	/**
	 * Puts the Limelight into driver mode, with the camera settings turned to driver mode (regular exposure)
	 * and the LEDs off.
	 * <p>
	 * Driver mode allows the driver to see through the Limelight camera and turns off the LEDs so it does not
	 * distract others on the field.
	 */
	public void enableDriverMode() {
		setCameraMode(LimelightCameraMode.Driver);
		setLedMode(LimelightLEDMode.Off);
	}
	
	public boolean isHasValidTarget() {
		return hasValidTarget;
	}
	
	public double getYawToTarget() {
		return yawToTarget;
	}
	
	public double getPitchToTarget() {
		return pitchToTarget;
	}
	
	public double getTargetArea() {
		return targetArea;
	}
	
	public double getTargetScreenRotation() {
		return targetScreenRotation;
	}
	
	public double getLimelightLatency() {
		return limelightLatency;
	}
	
	public double getBoxShortestSide() {
		return boxShortestSide;
	}
	
	public double getBoxLongestSide() {
		return boxLongestSide;
	}
	
	public double getBoxHorizontalSide() {
		return boxHorizontalSide;
	}
	
	public double getBoxVerticalSide() {
		return boxVerticalSide;
	}
	
	public double get3DCameraX() {
		return target3DCamera[0];
	}
	
	public double get3DCameraY() {
		return target3DCamera[1];
	}
	
	public double get3DCameraZ() {
		return target3DCamera[2];
	}
	
	public double get3DCameraPitch() {
		return target3DCamera[3];
	}
	
	public double get3DCameraYaw() {
		return target3DCamera[4];
	}
	
	public double get3DCameraRoll() {
		return target3DCamera[5];
	}
	
	public double[] get3DCamera() {
		return target3DCamera;
	}
	
	public double[] getTargetCornerX() {
		return targetCornerX;
	}
	
	public double[] getTargetCornerY() {
		return targetCornerY;
	}
}
