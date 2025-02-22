package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NinjasSimulatedController extends NinjasController {
    private double _maxVelocity;
    private double _maxAcceleration;
    private double _output = 0;
    private double _lastOutput = 0;
    private double _velocity = 0;
    private double _position = 0;

    private final TrapezoidProfile _profile;
    private final ProfiledPIDController _PIDFController;
    private final PIDController _PIDController;
    private boolean isCurrentlyPidfing = false;

    public NinjasSimulatedController(SimulatedControllerConstants constants) {
        super(constants.mainControllerConstants);

        switch (constants.motorType) {
            case KRAKEN, FALCON:
                _maxVelocity = 100;
                _maxAcceleration = 120;
                break;
            case NEO:
                _maxVelocity = 94;
                _maxAcceleration = 80;
                break;
            case NEO550:
                _maxVelocity = 183;
                _maxAcceleration = 1183;
                break;
            case CIM:
                _maxVelocity = 44;
                _maxAcceleration = 88;
                break;
            case KRAKEN_PRO, FALCON_PRO:
                _maxVelocity = 97;
                _maxAcceleration = 240;
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
    public double getCurrent() {
        return 0;
    }

    @Override
    public void setEncoder(double position) {
        _position = position;
    }

    @Override
    public void periodic() {
        switch (_constants.controlConstants.type) {
            case PROFILED_PID:
                isCurrentlyPidfing = true;

                if(_controlState == ControlState.POSITION)
                    _output = _PIDFController.calculate(getPosition());
                else if(_controlState == ControlState.VELOCITY)
                    _output = _PIDFController.calculate(getVelocity());
                break;

            case PID, TORQUE_CURRENT:
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

        if (!isCurrentlyPidfing && _controlState != ControlState.PERCENT_OUTPUT)
            _PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
        isCurrentlyPidfing = false;

        calculateKinematics();

        super.periodic();
    }

    private void calculateKinematics() {
        _output = MathUtil.clamp(_output, -1, 1);
        double dt = 0.02;
        double v0 = _velocity;

//        double accelerationDir = Math.signum(_output - _lastOutput);
        double velocityDir = Math.signum(v0);
        double outputDir = Math.signum(_output);
        double wantedVelocity = _output * _maxVelocity;

        double dynamicAccelerationLimiter;
        if((outputDir == velocityDir && Math.abs(wantedVelocity) >= Math.abs(v0)))
            dynamicAccelerationLimiter = _maxAcceleration * (1 - Math.pow(Math.abs(v0) / _maxVelocity, 2));
        else
            dynamicAccelerationLimiter = _maxAcceleration * 5;

        _velocity +=
            MathUtil.clamp(
                wantedVelocity - v0,
                -dynamicAccelerationLimiter * dt,
                dynamicAccelerationLimiter * dt);

        double a = (_velocity - v0) / dt;
        _position += v0 * dt + 0.5 * a * dt * dt;
        _lastOutput = _output;
    }
}
