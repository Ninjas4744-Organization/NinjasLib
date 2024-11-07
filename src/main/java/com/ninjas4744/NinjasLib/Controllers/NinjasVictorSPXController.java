package com.ninjas4744.NinjasLib.Controllers;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;

public class NinjasVictorSPXController extends NinjasController {
	private final VictorSPX _main;
	private final VictorSPX[] _followers;

	public NinjasVictorSPXController(MainControllerConstants constants) {
		super(constants);

		_main = new VictorSPX(constants.main.id);
		_main.configFactoryDefault();
		_main.setInverted(constants.main.inverted);

		_followers = new VictorSPX[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new VictorSPX(constants.followers[i].id);
			_followers[i].configFactoryDefault();
			_followers[i].follow(_main);
			_followers[i].setInverted(constants.followers[i].inverted ^ constants.main.inverted);
		}
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(VictorSPXControlMode.PercentOutput, percent);
	}

	@Override
	public void setPosition(double position) {
		throw new UnsupportedOperationException("PID not supported on VictorSPX");
	}

	@Override
	public void setVelocity(double velocity) {
		throw new UnsupportedOperationException("PID not supported on VictorSPX");
	}

	@Override
	public double getPosition() {
		throw new UnsupportedOperationException("No encoder on VictorSPX");
	}

	@Override
	public double getVelocity() {
		throw new UnsupportedOperationException("No encoder on VictorSPX");
	}

	@Override
	public double getOutput() {
		return _main.getMotorOutputPercent();
	}

	@Override
	public void setEncoder(double position) {
		throw new UnsupportedOperationException("No encoder on VictorSPX");
	}

	@Override
	public boolean atGoal() {
		throw new UnsupportedOperationException("PID not supported on VictorSPX");
	}
}
