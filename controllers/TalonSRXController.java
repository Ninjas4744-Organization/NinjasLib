package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.NinjasLib.dataclasses.RealControllerConstants;

public class TalonSRXController extends Controller {
	private final TalonSRX _main;
	private final TalonSRX[] _followers;

    public TalonSRXController(RealControllerConstants constants) {
		super(constants);

		_main = new TalonSRX(constants.main.id);
		_main.configFactoryDefault();
		_main.setInverted(constants.main.inverted);
		_main.configPeakCurrentLimit((int) constants.currentLimit);

		_main.config_kP(0, constants.controlConstants.P);
		_main.config_kI(0, constants.controlConstants.I);
		_main.config_kD(0, constants.controlConstants.D);
		_main.configMotionCruiseVelocity(
            constants.controlConstants.cruiseVelocity * constants.conversionFactor / 10);
        _main.configMotionAcceleration(constants.controlConstants.acceleration * constants.conversionFactor / 10);
        _main.configForwardSoftLimitEnable(constants.maxSoftLimit != Double.MAX_VALUE);
        _main.configReverseSoftLimitEnable(constants.minSoftLimit != Double.MIN_VALUE);
		_main.configForwardSoftLimitThreshold(constants.maxSoftLimit);
		_main.configReverseSoftLimitThreshold(constants.minSoftLimit);

		_followers = new TalonSRX[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new TalonSRX(constants.followers[i].id);
			_followers[i].configFactoryDefault();
			_followers[i].follow(_main);
			_followers[i].setInverted(constants.followers[i].inverted ^ constants.main.inverted);
		}
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(TalonSRXControlMode.PercentOutput, percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        switch (constants.controlConstants.type) {
			case PROFILE, PROFILED_PID:
                _main.set(TalonSRXControlMode.MotionMagic, position / constants.conversionFactor);
				break;

			case PID:
                _main.set(TalonSRXControlMode.Position, position / constants.conversionFactor);
				break;
		}
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        switch (constants.controlConstants.type) {
			case PROFILED_PID:
                _main.set(TalonSRXControlMode.MotionMagic, velocity / constants.conversionFactor);
				break;

			case PID:
                _main.set(TalonSRXControlMode.Velocity, velocity / constants.conversionFactor);
				break;

			case PROFILE:
				throw new UnsupportedOperationException("Velocity profile control not supported on TalonSRX");
		}
	}

	@Override
	public void stop() {
		_main.set(TalonSRXControlMode.PercentOutput, 0);
	}

	@Override
	public double getPosition() {
        return _main.getSelectedSensorPosition() * constants.conversionFactor;
	}

	@Override
	public double getVelocity() {
        return _main.getSelectedSensorVelocity() * constants.conversionFactor;
	}

	@Override
	public double getOutput() {
		return _main.getMotorOutputPercent();
	}

	@Override
	public double getCurrent() {
		return _main.getBusVoltage();
	}


	@Override
	public void setEncoder(double position) {
        _main.setSelectedSensorPosition(position / constants.conversionFactor);
	}
}
