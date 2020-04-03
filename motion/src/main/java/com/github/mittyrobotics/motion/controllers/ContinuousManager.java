package com.github.mittyrobotics.motion.controllers;

public class ContinuousManager {
    private double minInput, maxInput;
    private double inputRange;

    public ContinuousManager(double minInput, double maxInput){
        setInputRange(minInput, maxInput);
    }

    public ContinuousManager(){
        this(0, 0);
    }

    public void setInputRange(double minInput, double maxInput){
        this.minInput = minInput;
        this.maxInput = maxInput;
        inputRange = maxInput - minInput;
    }

    public double getMinInput() {
        return minInput;
    }

    public double getMaxInput() {
        return maxInput;
    }

    public boolean isEnabled() {
        return inputRange > 0;
    }

    public double getInputRange(){
        return inputRange;
    }

    public double getContinousError(double setpoint, double measurement){
        if(isEnabled() && measurement >= minInput && measurement <= maxInput){
            double error = (setpoint - measurement) % inputRange;
            if (Math.abs(error) > inputRange / 2) {
                if (error > 0) {
                    return error - inputRange;
                } else {
                    return error + inputRange;
                }
            }
        }
        return setpoint - measurement;
    }

    public double mapValue(double value){
        if(isEnabled()){
            value %= inputRange;
            if(value < 0){
                value += inputRange;
            }
            value += minInput;
        }
        return value;
    }
}