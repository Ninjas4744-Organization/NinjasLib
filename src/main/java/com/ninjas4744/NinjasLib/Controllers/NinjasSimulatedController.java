package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants.SmartControlType;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class NinjasSimulatedController extends NinjasController {
    private double _maxVelocity;
    private double _maxAcceleration;
    private double _output = 0;
    private double _velocity = 0;
    private double _position = 0;

    private SlewRateLimiter _accelerationLimiter;

    private final TrapezoidProfile _profile;
    private final ProfiledPIDController _PIDFController;
    private final PIDController _PIDController;
    private boolean isCurrentlyPiding = false;

    public NinjasSimulatedController(SimulatedControllerConstants constants) {
        super(constants.mainControllerConstants);

        switch (constants.motorType) {
            case KRAKEN:
                _maxVelocity = 100;
                _maxAcceleration = 100;
                break;
            case FALCON:
                _maxVelocity = 100;
                _maxAcceleration = 50;
                break;
            case NEO:
                _maxVelocity = 94;
                _maxAcceleration = 50;
                break;
            case NEO550:
                _maxVelocity = 183;
                _maxAcceleration = 6;
                break;
            case CIM:
                _maxVelocity = 44;
                _maxAcceleration = 12.5;
                break;
            case KRAKEN_PRO:
                _maxVelocity = 97;
                _maxAcceleration = 125;
                break;
            case FALCON_PRO:
                _maxVelocity = 97;
                _maxAcceleration = 63;
                break;
        }

        _accelerationLimiter = new SlewRateLimiter(_maxAcceleration);

        _profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
          constants.mainControllerConstants.controlConstants.CruiseVelocity,
          constants.mainControllerConstants.controlConstants.Acceleration));

        _PIDFController = new ProfiledPIDController(
          constants.mainControllerConstants.controlConstants.P,
          constants.mainControllerConstants.controlConstants.I,
          constants.mainControllerConstants.controlConstants.D,
          new TrapezoidProfile.Constraints(
            constants.mainControllerConstants.controlConstants.CruiseVelocity, constants.mainControllerConstants.controlConstants.Acceleration));
        _PIDFController.setIZone(constants.mainControllerConstants.controlConstants.IZone);

        _PIDController = new PIDController(
          constants.mainControllerConstants.controlConstants.P,
          constants.mainControllerConstants.controlConstants.I,
          constants.mainControllerConstants.controlConstants.D
        );
        _PIDController.setIZone(constants.mainControllerConstants.controlConstants.IZone);
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        _output = percent;
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        _PIDFController.setGoal(position);
        _PIDController.setSetpoint(position);
    }

    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        _PIDFController.setGoal(velocity);
        _PIDController.setSetpoint(velocity);
    }

    @Override
    public double getPosition() {
        return _position * _constants.encoderConversionFactor;
    }

    @Override
    public double getVelocity() {
        return _velocity * _constants.encoderConversionFactor;
    }

    @Override
    public double getOutput() {
        return _output;
    }

    @Override
    public void setEncoder(double position) {
        _position = position;
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
                break;

            case PID:
                isCurrentlyPiding = true;

                if(_controlState == ControlState.POSITION)
                    _output = _PIDController.calculate(getPosition());
                else if(_controlState == ControlState.VELOCITY)
                    _output = _PIDController.calculate(getVelocity());
                break;

            case PROFILE:
                if(_controlState == ControlState.POSITION)
                    _output = _profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getGoal(), 0))
                      .velocity * _constants.controlConstants.V / 12;
                else if(_controlState == ControlState.VELOCITY)
                    _output = _profile.calculate(
                      0.02,
                      new TrapezoidProfile.State(getPosition(), getVelocity()),
                      new TrapezoidProfile.State(getPosition(), getGoal()))
                      .velocity * _constants.controlConstants.V / 12;
                break;
        }

        if (!isCurrentlyPiding && _controlState != ControlState.PERCENT_OUTPUT)
            _PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
        isCurrentlyPiding = false;

        _output = MathUtil.clamp(_output, -1, 1);
        double dt = 0.02;
        double v0 = _velocity;
        _velocity = _accelerationLimiter.calculate(_output * _maxVelocity);
        double a = (_velocity - v0) / dt;
        _position += v0 * dt + 0.5 * a * dt * dt;
    }
}
