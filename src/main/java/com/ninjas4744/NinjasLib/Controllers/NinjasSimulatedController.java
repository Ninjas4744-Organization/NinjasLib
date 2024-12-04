package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants.SmartControlType;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class NinjasSimulatedController extends NinjasController {
    private DCMotorSim _main;

    private final TrapezoidProfile _profile;
    private final ProfiledPIDController _PIDFController;
    private final PIDController _PIDController;
    private boolean isCurrentlyPiding = false;
    private double _output = 0;

    public NinjasSimulatedController(SimulatedControllerConstants constants) {
        super(constants.mainControllerConstants);

        switch (constants.motorType) {
            case KRAKEN:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getKrakenX60(constants.mainControllerConstants.followers.length + 1));
                break;
            case FALCON:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getFalcon500(constants.mainControllerConstants.followers.length + 1));
                break;
            case NEO:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getNEO(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getNEO(constants.mainControllerConstants.followers.length + 1));
                break;
            case NEO550:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getNeo550(constants.mainControllerConstants.followers.length + 1));
                break;
            case CIM:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getCIM(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getCIM(constants.mainControllerConstants.followers.length + 1));
                break;
            case FALCON_FOC:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getFalcon500Foc(constants.mainControllerConstants.followers.length + 1));
                break;
            case KRAKEN_FOC:
                _main = new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(constants.mainControllerConstants.followers.length + 1), constants.motorTorque, constants.gearRatio),
                  DCMotor.getKrakenX60Foc(constants.mainControllerConstants.followers.length + 1));
                break;
        }

        _profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
          constants.mainControllerConstants.controlConstants.CruiseVelocity,
          constants.mainControllerConstants.controlConstants.Acceleration));

        _PIDFController = new ProfiledPIDController(
          constants.mainControllerConstants.controlConstants.P,
          constants.mainControllerConstants.controlConstants.I,
          constants.mainControllerConstants.controlConstants.D,
          new TrapezoidProfile.Constraints(
            constants.mainControllerConstants.controlConstants.CruiseVelocity, constants.mainControllerConstants.controlConstants.Acceleration));
        _PIDFController.setIZone(constants.mainControllerConstants.controlConstants.kIZone);

        _PIDController = new PIDController(
          constants.mainControllerConstants.controlConstants.kP,
          constants.mainControllerConstants.controlConstants.kI,
          constants.mainControllerConstants.controlConstants.kD
        );
        _PIDController.setIZone(constants.mainControllerConstants.controlConstants.kIZone);
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        _main.setInputVoltage(12 * percent);
        _output = percent;
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        _PIDFController.setGoal(position);
    }

    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        _PIDFController.setGoal(velocity);
    }

    @Override
    public double getPosition() {
        return _main.getAngularPositionRotations() * _constants.encoderConversionFactor;
    }

    @Override
    public double getVelocity() {
        return _main.getAngularVelocityRPM() / 60 * _constants.encoderConversionFactor;
    }

    @Override
    public double getOutput() {
        return _output;
    }

    @Override
    public void setEncoder(double position) {
        _main.setState(position * Math.PI * 2 / _constants.encoderConversionFactor, _main.getAngularVelocityRadPerSec());
    }

    @Override
    public void periodic() {
        super.periodic();

        switch (_constants.controlConstants.type) {
            case PROFILED_PID:
                isCurrentlyPiding = true;

                if(_controlState == ControlState.POSITION)
                    _output = _PIDFController.calculate(getPosition());
                else if(_controlState == ControlState.VELOCITY)
                    _output = _PIDFController.calculate(getVelocity());

                _main.setInputVoltage(12 * _output);
                break;

            case PID:
                isCurrentlyPiding = true;

                if(_controlState == ControlState.POSITION)
                    _output = _PIDController.calculate(getPosition());
                else if(_controlState == ControlState.VELOCITY)
                    _output = _PIDController.calculate(getVelocity());

                _main.setInputVoltage(12 * _output);
                break;

            case PROFILE:
                if(_controlState == ControlState.POSITION)
                    _output = _profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getGoal(), 0))
                      .velocity;
                else if(_controlState == ControlState.VELOCITY)
                    _output = _profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getPosition(), getGoal()))
                      .velocity;

                _main.setInputVoltage(_output * _constants.controlConstants.kV);
                break;
        }

        if (!isCurrentlyPiding && _controlState != ControlState.PERCENT_OUTPUT)
            _PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
        isCurrentlyPiding = false;

        _main.update(0.02);
    }
}
