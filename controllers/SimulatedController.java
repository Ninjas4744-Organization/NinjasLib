package frc.lib.NinjasLib.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;

public class SimulatedController extends Controller {
    private final TrapezoidProfile profile;
    private final ProfiledPIDController profiledPIDController;
    private final PIDController PIDController;
    private boolean isCurrentlyProfiling = false;
    private DCMotorSim motorSim;
    private double lastVelocity;

    public SimulatedController(ControllerConstants constants) {
        super(constants.real);

        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.motorType, 0.001, constants.real.control.gearRatio), constants.motorType, 0.0005, 0.0005);

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            constants.real.control.controlConstants.cruiseVelocity,
            constants.real.control.controlConstants.acceleration));

        profiledPIDController = new ProfiledPIDController(
            constants.real.control.controlConstants.P,
            constants.real.control.controlConstants.I,
            constants.real.control.controlConstants.D,
          new TrapezoidProfile.Constraints(
              constants.real.control.controlConstants.cruiseVelocity, constants.real.control.controlConstants.acceleration));
        profiledPIDController.setIZone(constants.real.control.controlConstants.IZone);

        PIDController = new PIDController(
            constants.real.control.controlConstants.P,
            constants.real.control.controlConstants.I,
            constants.real.control.controlConstants.D
        );
        PIDController.setIZone(constants.real.control.controlConstants.IZone);
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
        return motorSim.getAngularPositionRotations() * constants.control.conversionFactor;
    }

    @Override
    public double getVelocity() {
        return motorSim.getAngularVelocityRPM() / 60 * constants.control.conversionFactor;
    }

    @Override
    public double getAcceleration() {
        double acc = (getVelocity() - lastVelocity) / 0.02;
        lastVelocity = getVelocity();
        return acc;
    }

    @Override
    public double getOutput() {
        return motorSim.getInputVoltage() / 12; // TODO FIX
    }

    @Override
    public double getCurrent() {
        return motorSim.getCurrentDrawAmps(); // TODO FIX
    }

    @Override
    public void setEncoder(double position) {
        motorSim.setState(position, motorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void periodic() {
        switch (constants.control.controlConstants.type) {
            case PROFILED_PID:
                isCurrentlyProfiling = true;

                if (controlState == ControlState.POSITION)
                    motorSim.setInputVoltage(profiledPIDController.calculate(getPosition()));
                else if (controlState == ControlState.VELOCITY)
                    motorSim.setInputVoltage(profiledPIDController.calculate(getVelocity()));
                break;

            case PID, TORQUE_CURRENT:
                if (controlState == ControlState.POSITION)
                    motorSim.setInputVoltage(PIDController.calculate(getPosition()));
                else if (controlState == ControlState.VELOCITY)
                    motorSim.setInputVoltage(PIDController.calculate(getVelocity()));
                break;

            case PROFILE:
                if (controlState == ControlState.POSITION)
                    motorSim.setInputVoltage(profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getGoal(), 0))
                        .velocity * constants.control.controlConstants.V);
                else if (controlState == ControlState.VELOCITY)
                    motorSim.setInputVoltage(profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getPosition(), getGoal()))
                        .velocity * constants.control.controlConstants.V);
                break;
        }

        if (!isCurrentlyProfiling && controlState != ControlState.PERCENT_OUTPUT)
            profiledPIDController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
        isCurrentlyProfiling = false;

        if (getPosition() >= constants.softLimits.max) {
            stop();
            setEncoder(constants.softLimits.max);
        }

        if (getPosition() <= constants.softLimits.min) {
            stop();
            setEncoder(constants.softLimits.min);
        }

        motorSim.update(0.02);

        super.periodic();
    }
}
