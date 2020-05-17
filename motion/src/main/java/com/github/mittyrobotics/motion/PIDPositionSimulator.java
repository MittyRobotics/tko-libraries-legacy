/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.motion.controllers.PIDFController;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.scene.text.Text;
import javafx.stage.Stage;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

public class PIDPositionSimulator extends Application {
    private double p = 0, i = 0, d = 0;
    private double position = 0;
    private double setpoint = 0;
    private double maxSpeed = 1;
    private double maxPercent = 1;
    private AtomicBoolean shouldRun;
    private double ticksPerPixel = 1;
    private double maxAcceleration = 1;
    private double scalar = 500;
    private double velocity = 0;

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage stage) throws Exception {
        shouldRun = new AtomicBoolean(false);
        Pane root = new Pane();
        Scene scene = new Scene(root, 700, 700);
        stage.setScene(scene);
        stage.show();
        Button startButton = new Button("Start");
        startButton.setLayoutX(0);
        startButton.setLayoutY(0);
        Rectangle robot = new Rectangle();
        robot.setWidth(70);
        robot.setHeight(50);
        robot.setY(250);
        robot.setFill(Color.GREEN);
        PIDFController controller = new PIDFController(p, i, d);

        Text vArea = new Text();
        vArea.setX(450);
        vArea.setY(20);
        vArea.setText("Velocity: ");
        root.getChildren().addAll(vArea);
        Text vField = new Text();
        vField.setY(20);
        vField.setX(510);
        vField.setText(Double.toString(velocity));
        root.getChildren().addAll(vField);

        Text aArea = new Text();
        aArea.setX(450);
        aArea.setY(50);
        aArea.setText("Max Acceleration: ");
        root.getChildren().addAll(aArea);
        TextField aField = new TextField();
        aField.setLayoutY(30);
        aField.setLayoutX(560);
        aField.setMaxWidth(50);
        aField.setText(Double.toString(maxAcceleration));
        root.getChildren().addAll(aField);

        Text pArea = new Text();
        pArea.setX(0);
        pArea.setY(50);
        pArea.setText("P: ");
        root.getChildren().addAll(pArea);
        TextField pField = new TextField();
        pField.setLayoutY(30);
        pField.setLayoutX(20);
        pField.setMaxWidth(50);
        pField.setText(Double.toString(p));
        root.getChildren().addAll(pField);

        Text iArea = new Text();
        iArea.setX(0);
        iArea.setY(80);
        iArea.setText("I: ");
        root.getChildren().addAll(iArea);
        TextField iField = new TextField();
        iField.setLayoutY(60);
        iField.setLayoutX(20);
        iField.setMaxWidth(50);
        iField.setText(Double.toString(i));
        root.getChildren().addAll(iField);

        Text dArea = new Text();
        dArea.setX(0);
        dArea.setY(110);
        dArea.setText("D: ");
        root.getChildren().addAll(dArea);
        TextField dField = new TextField();
        dField.setLayoutY(90);
        dField.setLayoutX(20);
        dField.setMaxWidth(50);
        dField.setText(Double.toString(d));
        root.getChildren().addAll(dField);

        Text setpointArea = new Text();
        setpointArea.setX(70);
        setpointArea.setY(20);
        setpointArea.setText("Setpoint: ");
        root.getChildren().addAll(setpointArea);
        TextField setpointField = new TextField();
        setpointField.setLayoutY(0);
        setpointField.setLayoutX(130);
        setpointField.setMaxWidth(80);
        setpointField.setText(Double.toString(setpoint));
        root.getChildren().addAll(setpointField);

        Text positionArea = new Text();
        positionArea.setX(70);
        positionArea.setY(50);
        positionArea.setText("Position: ");
        root.getChildren().addAll(positionArea);
        Text positionField = new Text();
        positionField.setY(50);
        positionField.setX(130);
        positionField.setText(Double.toString(position));
        root.getChildren().addAll(positionField);

        Text maxSpeedArea = new Text();
        maxSpeedArea.setX(70);
        maxSpeedArea.setY(80);
        maxSpeedArea.setText("Max Speed: ");
        root.getChildren().addAll(maxSpeedArea);
        TextField maxSpeedField = new TextField();
        maxSpeedField.setLayoutY(60);
        maxSpeedField.setLayoutX(140);
        maxSpeedField.setMaxWidth(80);
        maxSpeedField.setText(Double.toString(maxSpeed));
        root.getChildren().addAll(maxSpeedField);

        Text maxPercentArea = new Text();
        maxPercentArea.setX(70);
        maxPercentArea.setY(110);
        maxPercentArea.setText("Max Percent: ");
        root.getChildren().addAll(maxPercentArea);
        TextField maxPercentField = new TextField();
        maxPercentField.setLayoutY(90);
        maxPercentField.setLayoutX(150);
        maxPercentField.setMaxWidth(80);
        maxPercentField.setText(Double.toString(maxPercent));
        root.getChildren().addAll(maxPercentField);

        Slider slider = new Slider(.1, 10, ticksPerPixel);
        slider.setBlockIncrement(.1);
        slider.setMajorTickUnit(1);
        slider.setMajorTickUnit(.1);
        slider.setMajorTickUnit(.5);
        slider.setMajorTickUnit(2);
        slider.setMajorTickUnit(10);
        slider.setShowTickLabels(true);
        slider.setShowTickMarks(true);
        slider.setLayoutX(250);
        slider.setLayoutY(50);
        root.getChildren().addAll(slider);

        Text sliderLabel = new Text();
        sliderLabel.setX(280);
        sliderLabel.setY(100);
        sliderLabel.setText("Scaling: " + slider.getValue());
        root.getChildren().addAll(sliderLabel);

        Text scalarText = new Text();
        scalarText.setX(450);
        scalarText.setY(450);
        scalarText.setText(Double.toString(scalar));
        root.getChildren().addAll(scalarText);

        Button reset = new Button();
        reset.setText("Reset");
        reset.setLayoutX(400);
        //root.getChildren().addAll(reset);
        reset.setOnMouseClicked(mouseEvent -> {
            position = 0;
            controller.setP(0);
            controller.setI(0);
            controller.setD(0);
            controller.setSetpoint(0);
            maxPercent = 1;
            maxSpeed = 1;
            ticksPerPixel = 1;
            scalar = 500;
            pField.setText(Double.toString(controller.getkP()));
            iField.setText(Double.toString(controller.getkI()));
            dField.setText(Double.toString(controller.getkD()));
            setpointField.setText(Double.toString(controller.getSetpoint()));
            maxPercentField.setText(Double.toString(maxPercent));
            maxSpeedField.setText(Double.toString(maxSpeed));
            scalarText.setText(Double.toString(scalar));
            slider.setValue(1);
            sliderLabel.setText("Scaling: " + slider.getValue());
        });

        controller.setOutputRange(-maxPercent, maxPercent);
        controller.setSetpoint(setpoint);
        root.getChildren().addAll(startButton);
        root.getChildren().addAll(robot);
        startButton.setOnMouseClicked(mouseEvent -> {
            if (shouldRun.get()) {
                shouldRun.set(false);
                startButton.setText("Start");
            } else {
                shouldRun.set(true);
                controller.setP(Double.parseDouble(pField.getText()));
                controller.setI(Double.parseDouble(iField.getText()));
                controller.setD(Double.parseDouble(dField.getText()));
                controller.setSetpoint(setpoint = Double.parseDouble(setpointField.getText()));
                maxSpeed = Double.parseDouble(maxSpeedField.getText());
                maxPercent = Double.parseDouble(maxPercentField.getText());
                controller.setOutputRange(-maxPercent, maxPercent);
                maxAcceleration = Double.parseDouble(aField.getText());
                startButton.setText("Stop");
            }
        });
        Timer clockTimer = new Timer();
        clockTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (shouldRun.get()) {
                    if (Math.abs(velocity - controller.calculate(position) * maxSpeed) <
                            maxAcceleration * 0.02 * maxSpeed) {
                        velocity = controller.calculate(position) * maxSpeed;
                    } else if (velocity < controller.calculate(position) * maxSpeed) {
                        velocity += maxAcceleration * 0.02 * maxSpeed;
                    } else {
                        velocity -= maxAcceleration * 0.02 * maxSpeed;
                    }
                    position += velocity * .02;
                    robot.setX(((2 * position / ticksPerPixel - robot.getWidth()) / 2));
                    positionField.setText(Double.toString(position));
                    vField.setText(Double.toString(velocity));
                }
                ticksPerPixel = Math.round(slider.getValue() * 10) / 10.0;
                sliderLabel.setText("Slider " + Math.round(slider.getValue() * 10) / 10.0);
                scalar = 500 * ticksPerPixel;
                scalarText.setText(Double.toString(Math.round(scalar * 10) / 10.0));
            }
        }, 0, 20);
    }

    private void updateValues() {

    }
}