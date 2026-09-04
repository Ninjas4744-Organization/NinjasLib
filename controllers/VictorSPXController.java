package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

/**
 * {@link Controller} implementation that wraps a CTRE VictorSPX motor controller via the legacy
 * Phoenix 5 API. The VictorSPX has no built-in encoder or current sensing and no onboard
 * closed-loop control, so it only supports open-loop percent output ({@link #setPercent(double)});
 * every position/velocity/current/PID-related method throws {@link UnsupportedOperationException}.
 * Typically used for simple, unsensored mechanisms like intake or feeder rollers.
 */
public class VictorSPXController extends Controller {
	private final VictorSPX main;
	private final VictorSPX[] followers;

    /**
     * Factory-resets and configures the main VictorSPX (inversion), then factory-resets and
     * configures each follower VictorSPX to follow it.
     *
     * @param constants the controller configuration
     */
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

	/**
	 * Drives the main VictorSPX directly in open-loop percent output. This is the only supported
	 * control mode on this hardware.
	 *
	 * @param percent how much to power the motor, between -1 and 1
	 */
	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		main.set(VictorSPXControlMode.PercentOutput, percent);
	}

	/**
	 * @param position unused
	 * @throws UnsupportedOperationException always; the VictorSPX has no onboard PID
	 */
	@Override
	public void setPosition(double position) {
		throw new UnsupportedOperationException("PID not supported on VictorSPX");
	}

	/**
	 * @param velocity unused
	 * @throws UnsupportedOperationException always; the VictorSPX has no onboard PID
	 */
	@Override
	public void setVelocity(double velocity) {
		throw new UnsupportedOperationException("PID not supported on VictorSPX");
	}

	/** Stops the main VictorSPX (and its followers) by commanding zero percent output. */
	@Override
	public void stop() {
		super.stop();
		main.set(VictorSPXControlMode.PercentOutput, 0);
	}

	/** @throws UnsupportedOperationException always; the VictorSPX has no encoder */
	@Override
	public double getPosition() {
		throw new UnsupportedOperationException("No encoder on VictorSPX");
	}

	/** @throws UnsupportedOperationException always; the VictorSPX has no encoder */
	@Override
	public double getVelocity() {
		throw new UnsupportedOperationException("No encoder on VictorSPX");
	}

    /** @throws UnsupportedOperationException always; the VictorSPX has no encoder */
    @Override
    public double getAcceleration() {
        throw new UnsupportedOperationException("No encoder on VictorSPX");
    }

	/** @return the main VictorSPX's applied motor output, between -1 and 1 */
	@Override
	public double getOutput() {
		return main.getMotorOutputPercent();
	}

	/** @throws UnsupportedOperationException always; the VictorSPX has no current sensing */
	@Override
	public double getSupplyCurrent() {
        throw new UnsupportedOperationException("No current handling on VictorSPX");
	}

	/** @throws UnsupportedOperationException always; the VictorSPX has no current sensing */
	@Override
	public double getStatorCurrent() {
        throw new UnsupportedOperationException("No current handling on VictorSPX");
	}

	/**
	 * @param position unused
	 * @throws UnsupportedOperationException always; the VictorSPX has no encoder
	 */
	@Override
	public void setEncoder(double position) {
		throw new UnsupportedOperationException("No encoder on VictorSPX");
	}

	/** @throws UnsupportedOperationException always; the VictorSPX has no onboard PID or encoder to check a goal against */
	@Override
	public boolean atGoal() {
		throw new UnsupportedOperationException("PID not supported on VictorSPX");
	}
}
