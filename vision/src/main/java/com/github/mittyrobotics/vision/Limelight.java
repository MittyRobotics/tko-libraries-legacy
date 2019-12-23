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
	
	private int pipelineID = 0;
	
	private Limelight() {
	
	}
	
	public void initDefaultLimelightSettings(){
		setPipeline(0);
		setLedMode(LimelightLEDMode.On);
		setCameraMode(LimelightCameraMode.Vision);
		setStreamMode(LimelightStreamMode.Secondary);
		setSnapshotMode(LimelightSnapshotMode.Off);
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
		this.pipelineID = pipelineID;
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
	
}
