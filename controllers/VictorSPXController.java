package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

public class VictorSPXController extends Controller {
	private final VictorSPX main;
	private final VictorSPX[] followers;

    public VictorSPXController(RealControllerConstants constants) {
		super(constants);

		main = new VictorSPX(constants.base.main.id);
		main.configFactoryDefault();
		main.setInverted(constants.base.main.inverted);

		followers = new VictorSPX[constants.base.followers.length];
		for (int i = 0; i < followers.length; i++) {
			followers[i] = new VictorSPX(constants.base.followers[i].id);
			followers[i].configFactoryDefault();
			followers[i].follow(main);
			followers[i].setInverted(constants.base.followers[i].inverted ^ constants.base.main.inverted);
		}
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		main.set(VictorSPXControlMode.PercentOutput, percent);
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
	public void stop() {
		super.stop();
		main.set(VictorSPXControlMode.PercentOutput, 0);
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
    public double getAcceleration() {
        throw new UnsupportedOperationException("No encoder on VictorSPX");
    }

	@Override
	public double getOutput() {
		return main.getMotorOutputPercent();
	}

	@Override
	public double getSupplyCurrent() {
        throw new UnsupportedOperationException("No current handling on VictorSPX");
	}

	@Override
	public double getStatorCurrent() {
        throw new UnsupportedOperationException("No current handling on VictorSPX");
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
