package com.ninjas4744.lib.Controllers;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ninjas4744.lib.DataClasses.MainControllerConstants;

public class NinjasTalonSRXController extends NinjasController {
	private final TalonSRX _main;
	private final TalonSRX[] _followers;

	public NinjasTalonSRXController(MainControllerConstants constants) {
		super(constants);

		_main = new TalonSRX(constants.main.id);
		_main.configFactoryDefault();
		_main.setInverted(constants.main.inverted);
		_main.configPeakCurrentLimit((int) constants.currentLimit);

		_main.config_kP(0, constants.PIDFConstants.kP);
		_main.config_kI(0, constants.PIDFConstants.kI);
		_main.config_kD(0, constants.PIDFConstants.kD);
		_main.config_kF(0, constants.PIDFConstants.kF);
		_main.configMotionCruiseVelocity(constants.PIDFConstants.kCruiseVelocity * constants.encoderConversionFactor / 10);
		_main.configMotionAcceleration(constants.PIDFConstants.kAcceleration * constants.encoderConversionFactor / 10);

		_main.configForwardSoftLimitEnable(constants.isMaxSoftLimit);
		_main.configReverseSoftLimitEnable(constants.isMinSoftLimit);
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

		switch (_controlState) {
			case PIDF_POSITION:
				_main.set(TalonSRXControlMode.MotionMagic, position / _constants.encoderConversionFactor);
				break;

			case PID_POSITION:
				_main.set(TalonSRXControlMode.Position, position / _constants.encoderConversionFactor);
				break;

			case FF_POSITION:
				throw new UnsupportedOperationException("Feedforward control not supported on Talon SRX");
		}
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		switch (_controlState) {
			case PIDF_VELOCITY:
				throw new UnsupportedOperationException("PIDF control for velocity not supported on Talon SRX");

			case PID_VELOCITY:
				_main.set(TalonSRXControlMode.Velocity, _constants.encoderConversionFactor);
				break;

			case FF_VELOCITY:
				throw new UnsupportedOperationException("Feedforward control not supported on Talon SRX");
		}
	}

	@Override
	public double getPosition() {
		return _main.getSelectedSensorPosition() * _constants.encoderConversionFactor;
	}

	@Override
	public double getVelocity() {
		return _main.getSelectedSensorVelocity() * _constants.encoderConversionFactor;
	}

	@Override
	public double getOutput() {
		return _main.getMotorOutputPercent();
	}

	@Override
	public void setEncoder(double position) {
		_main.setSelectedSensorPosition(position / _constants.encoderConversionFactor);
	}
}
