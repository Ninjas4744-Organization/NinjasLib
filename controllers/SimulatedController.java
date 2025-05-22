package frc.lib.NinjasLib.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.NinjasLib.dataclasses.ControllerConstants;

public class SimulatedController extends Controller {
    private final TrapezoidProfile profile;
    private final ProfiledPIDController profiledPIDController;
    private final PIDController PIDController;
    private boolean isCurrentlyProfiling = false;
    private DCMotorSim motorSim;

    public SimulatedController(ControllerConstants constants) {
        super(constants.real);

        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.motorType, 0.001, constants.real.gearRatio), constants.motorType, 0.002, 0.002);

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            constants.real.controlConstants.cruiseVelocity,
            constants.real.controlConstants.acceleration));

        profiledPIDController = new ProfiledPIDController(
            constants.real.controlConstants.P,
            constants.real.controlConstants.I,
            constants.real.controlConstants.D,
          new TrapezoidProfile.Constraints(
              constants.real.controlConstants.cruiseVelocity, constants.real.controlConstants.acceleration));
        profiledPIDController.setIZone(constants.real.controlConstants.IZone);

        PIDController = new PIDController(
            constants.real.controlConstants.P,
            constants.real.controlConstants.I,
            constants.real.controlConstants.D
        );
        PIDController.setIZone(constants.real.controlConstants.IZone);
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        motorSim.setInputVoltage(percent * 12);
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        profiledPIDController.setGoal(position);
        PIDController.setSetpoint(position);
    }

    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        profiledPIDController.setGoal(velocity);
        PIDController.setSetpoint(velocity);
    }

    @Override
    public void stop() {
        motorSim.setInputVoltage(0);
    }

    @Override
    public double getPosition() {
        return motorSim.getAngularPositionRotations() * constants.conversionFactor;
    }

    @Override
    public double getVelocity() {
        return motorSim.getAngularVelocityRPM() / 60 * constants.conversionFactor;
    }

    @Override
    public double getOutput() {
        return motorSim.getOutput(0);
    }

    @Override
    public double getCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setEncoder(double position) {
        motorSim.setState(position / constants.conversionFactor, motorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void periodic() {
        switch (constants.controlConstants.type) {
            case PROFILED_PID:
                isCurrentlyProfiling = true;

                if (controlState == ControlState.POSITION)
                    motorSim.setInputVoltage(profiledPIDController.calculate(getPosition()) * 12);
                else if (controlState == ControlState.VELOCITY)
                    motorSim.setInputVoltage(profiledPIDController.calculate(getVelocity()) * 12);
                break;

            case PID, TORQUE_CURRENT:
                if (controlState == ControlState.POSITION)
                    motorSim.setInputVoltage(PIDController.calculate(getPosition()) * 12);
                else if (controlState == ControlState.VELOCITY)
                    motorSim.setInputVoltage(PIDController.calculate(getVelocity()) * 12);
                break;

            case PROFILE:
                if (controlState == ControlState.POSITION)
                    motorSim.setInputVoltage(profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getGoal(), 0))
                        .velocity * constants.controlConstants.V);
                else if (controlState == ControlState.VELOCITY)
                    motorSim.setInputVoltage(profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getPosition(), getGoal()))
                        .velocity * constants.controlConstants.V);
                break;
        }

        if (!isCurrentlyProfiling && controlState != ControlState.PERCENT_OUTPUT)
            profiledPIDController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
        isCurrentlyProfiling = false;

        motorSim.update(0.02);

        super.periodic();
    }

//    private void calculateKinematics() {
//        _output = MathUtil.clamp(_output, -1, 1);
//        double dt = 0.02;
//        double v0 = _velocity;
//
////        double accelerationDir = Math.signum(_output - _lastOutput);
//        double velocityDir = Math.signum(v0);
//        double outputDir = Math.signum(_output);
//        double wantedVelocity = _output * _maxVelocity;
//
//        double dynamicAccelerationLimiter;
//        if((outputDir == velocityDir && Math.abs(wantedVelocity) >= Math.abs(v0)))
//            dynamicAccelerationLimiter = _maxAcceleration * (1 - Math.pow(Math.abs(v0) / _maxVelocity, 2));
//        else
//            dynamicAccelerationLimiter = _maxAcceleration * 5;
//
//        _velocity +=
//            MathUtil.clamp(
//                wantedVelocity - v0,
//                -dynamicAccelerationLimiter * dt,
//                dynamicAccelerationLimiter * dt);
//
//        double a = (_velocity - v0) / dt;
//        _position += v0 * dt + 0.5 * a * dt * dt;
//        _lastOutput = _output;
//    }
}
