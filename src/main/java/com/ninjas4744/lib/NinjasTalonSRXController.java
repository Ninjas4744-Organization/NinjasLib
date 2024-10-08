package com.ninjas4744.lib;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ninjas4744.lib.data.MainControllerConstants;

public class NinjasTalonSRXController extends NinjasController {
	private TalonSRX _main;
	private TalonSRX[] _followers;

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
		_main.configMotionCruiseVelocity(constants.PIDFConstants.kCruiseVelocity / 10);
		_main.configMotionAcceleration(constants.PIDFConstants.kAcceleration / 10);

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
				_main.set(TalonSRXControlMode.Velocity, (_constants.encoderConversionFactor / 60));
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
		return _main.getSelectedSensorVelocity() * _constants.encoderConversionFactor / 60;
	}

	@Override
	public double getOutput() {
		return _main.getMotorOutputPercent();
	}

	@Override
	public void setEncoder(double position) {
		_main.setSelectedSensorPosition(position / _constants.encoderConversionFactor);
	}

	@Override
	public boolean atGoal() {
		if (_controlState == ControlState.PIDF_POSITION)
			return Math.abs(getGoal() - getPosition()) < _constants.positionGoalTolerance;
		else if (_controlState == ControlState.PIDF_VELOCITY)
			return Math.abs(getGoal() - getVelocity()) < _constants.velocityGoalTolerance;

		return false;
	}
}
