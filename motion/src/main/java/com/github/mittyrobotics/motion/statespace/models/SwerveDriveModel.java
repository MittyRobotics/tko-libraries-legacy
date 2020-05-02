package com.github.mittyrobotics.motion.statespace.models;

import com.github.mittyrobotics.datatypes.motion.SwerveDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.SwerveDriveState;
import com.github.mittyrobotics.datatypes.motion.SwerveModuleState;
import com.github.mittyrobotics.motion.statespace.motors.Motor;

public class SwerveDriveModel {
    private final Motor wheelMotor;
    private final double mass;
    private final double length;
    private final double width;
    private final double chassisMomentOfInertia;
    private final double wheelGearReduction;
    private final double wheelRadius;
    private SwerveDriveState position = new SwerveDriveState();
    private SwerveDriveState velocity = new SwerveDriveState();
    private SwerveDriveState acceleration = new SwerveDriveState();
    private final FlywheelModel frModule;
    private final FlywheelModel flModule;
    private final FlywheelModel blModule;
    private final FlywheelModel brModule;
    private final PulleyModel frWheelModel;
    private final PulleyModel flWheelModel;
    private final PulleyModel blWheelModel;
    private final PulleyModel brWheelModel;

    public SwerveDriveModel(Motor wheelMotor, Motor steerMotor, double mass, double length, double width,
                            /*double chassisMomentOfInertia,*/ double steerMomentOfInertia, double wheelGearReduction,
                            double steerGearReduction,
                            double wheelRadius) {
        this.wheelMotor = wheelMotor;
        this.mass = mass;
        this.length = length;
        this.width = width;
        this.chassisMomentOfInertia = 0;
        this.wheelGearReduction = wheelGearReduction;
        this.wheelRadius = wheelRadius;
        this.frModule = new FlywheelModel(steerMotor, steerMomentOfInertia, steerGearReduction, 12);
        this.flModule = new FlywheelModel(steerMotor, steerMomentOfInertia, steerGearReduction, 12);
        this.blModule = new FlywheelModel(steerMotor, steerMomentOfInertia, steerGearReduction, 12);
        this.brModule = new FlywheelModel(steerMotor, steerMomentOfInertia, steerGearReduction, 12);
        this.frWheelModel = new PulleyModel(wheelMotor, mass/4, wheelGearReduction, wheelRadius, 12);
        this.flWheelModel = new PulleyModel(wheelMotor, mass/4, wheelGearReduction, wheelRadius, 12);
        this.blWheelModel = new PulleyModel(wheelMotor, mass/4, wheelGearReduction, wheelRadius, 12);
        this.brWheelModel = new PulleyModel(wheelMotor, mass/4, wheelGearReduction, wheelRadius, 12);
    }

    public void updateModel(SwerveModuleState frVoltages, SwerveModuleState flVoltages,
                            SwerveModuleState blVoltages, SwerveModuleState brVoltages, double deltaTime) {
        //Define variables
        double G = wheelGearReduction;
        double Kt = wheelMotor.getKt();
        double Kv = wheelMotor.getKv();
        double R = wheelMotor.getResistance();
        double r = wheelRadius;
        double vFr = frVoltages.getWheelState();
        double VFr = velocity.getFrState().getWheelState();
        double vFl = flVoltages.getWheelState();
        double VFl = velocity.getFlState().getWheelState();
        double vBl = blVoltages.getWheelState();
        double VBl = velocity.getBlState().getWheelState();
        double vBr = brVoltages.getWheelState();
        double VBr = velocity.getBrState().getWheelState();
        double J = chassisMomentOfInertia;
        double w = width;
        double l = length;
        double m = mass;
        double prevVelY = velocity.getVelY();
        double prevVelX = velocity.getVelX();
        double dt = deltaTime;
        double wheelbaseRadius = Math.sqrt(Math.pow(length / 2, 2) + Math.pow(width / 2, 2));

        //Update module models
        frModule.updateModel(frVoltages.getSteerState(), deltaTime);
        flModule.updateModel(flVoltages.getSteerState(), deltaTime);
        blModule.updateModel(blVoltages.getSteerState(), deltaTime);
        brModule.updateModel(brVoltages.getSteerState(), deltaTime);
        frWheelModel.updateModel(frVoltages.getWheelState(), deltaTime);
        flWheelModel.updateModel(flVoltages.getWheelState(), deltaTime);
        blWheelModel.updateModel(blVoltages.getWheelState(), deltaTime);
        brWheelModel.updateModel(brVoltages.getWheelState(), deltaTime);

        //Swerve modules diagram:
        //2-------1
        //|       |
        //|       |
        //3-------4

        double angle1 = frModule.getPosition();
        double angle2 = flModule.getPosition();
        double angle3 = blModule.getPosition();
        double angle4 = brModule.getPosition();

        double C1 = -(G * G * Kt) / (Kv * R * (r * r));
        double C2 = (G * Kt) / (R * r);

        double F1 = C1 * vFr + C2 * VFr;
        double F2 = C1 * vFl + C2 * VFl;
        double F3 = C1 * vBl + C2 * VBl;
        double F4 = C1 * vBr + C2 * VBr;

        double vDotX = 0;
        double vDotY = 0;
        double vDotAngular = 0;

//        acceleration = new SwerveDriveState(vDotX, vDotY, vDotAngular);
        velocity = SwerveDriveKinematics.solveForwardKinematics(l, w, frWheelModel.getVelocity(), flWheelModel.getVelocity(), blWheelModel.getVelocity(), brWheelModel.getVelocity(), angle1, angle2, angle3, angle4);
        position = new SwerveDriveState(position.getVelX() + velocity.getVelX()*deltaTime, position.getVelY() + velocity.getVelY()*deltaTime, 0);
    }

    public SwerveDriveState getPosition() {
        return position;
    }

    public SwerveDriveState getVelocity() {
        return velocity;
    }

    public SwerveDriveState getAcceleration() {
        return acceleration;
    }

    public SwerveDriveState getSwervePositionState() {
        SwerveDriveState wheelStates = SwerveDriveKinematics
                .solveInverseKinematics(length, width, getVelocity().getVelX(), getVelocity().getVelY(),
                        getVelocity().getAngularVel());
        return new SwerveDriveState(
                new SwerveModuleState(wheelStates.getFrState().getWheelState(), getFrModule().getPosition()),
                new SwerveModuleState(wheelStates.getFlState().getWheelState(), getFlModule().getPosition()),
                new SwerveModuleState(wheelStates.getBlState().getWheelState(), getBlModule().getPosition()),
                new SwerveModuleState(wheelStates.getBrState().getWheelState(), getBrModule().getPosition())
        );
    }

    public FlywheelModel getFrModule() {
        return frModule;
    }

    public FlywheelModel getFlModule() {
        return flModule;
    }

    public FlywheelModel getBlModule() {
        return blModule;
    }

    public FlywheelModel getBrModule() {
        return brModule;
    }
}
