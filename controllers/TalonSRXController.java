package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

/**
 * {@link Controller} implementation that wraps a CTRE TalonSRX motor controller via the legacy
 * Phoenix 5 API. Closed-loop control (PID/Motion Magic) runs onboard the TalonSRX firmware, in
 * raw sensor units; the 10 in the Motion Magic conversions below accounts for Phoenix 5 expressing
 * cruise velocity/acceleration per 100ms rather than per second. Velocity profile ({@code PROFILE})
 * control is not supported by this hardware.
 */
public class TalonSRXController extends Controller {
    private final TalonSRX main;
    private final TalonSRX[] followers;
    private double lastVelocity;

    /**
     * Factory-resets and configures the main TalonSRX (inversion, peak current limit, PID gains,
     * Motion Magic cruise velocity/acceleration, and soft limits), then factory-resets and
     * configures each follower TalonSRX to follow it.
     *
     * @param constants the controller configuration
     */
    public TalonSRXController(RealControllerConstants constants) {
		super(constants);

        main = new TalonSRX(constants.base.main.id);
        main.configFactoryDefault();
        main.setInverted(constants.base.main.inverted);
        main.configPeakCurrentLimit((int) constants.base.statorCurrentLimit);

        main.config_kP(0, constants.control.controlConstants.P);
        main.config_kI(0, constants.control.controlConstants.I);
        main.config_kD(0, constants.control.controlConstants.D);
        main.configMotionCruiseVelocity(
            constants.control.controlConstants.cruiseVelocity * constants.control.conversionFactor / 10);
        main.configMotionAcceleration(constants.control.controlConstants.acceleration * constants.control.conversionFactor / 10);
        main.configForwardSoftLimitEnable(constants.softLimits.max != Double.POSITIVE_INFINITY);
        main.configReverseSoftLimitEnable(constants.softLimits.min != Double.NEGATIVE_INFINITY);
        main.configForwardSoftLimitThreshold(constants.softLimits.max);
        main.configReverseSoftLimitThreshold(constants.softLimits.min);

        followers = new TalonSRX[constants.base.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new TalonSRX(constants.base.followers[i].id);
            followers[i].configFactoryDefault();
            followers[i].follow(main);
            followers[i].setInverted(constants.base.followers[i].inverted ^ constants.base.main.inverted);
		}
	}

	/**
	 * Drives the main TalonSRX directly in open-loop percent output.
	 *
	 * @param percent how much to power the motor, between -1 and 1
	 */
	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

        main.set(TalonSRXControlMode.PercentOutput, percent);
	}

	/**
	 * Commands the main TalonSRX's onboard closed loop to the given position, converted to raw
	 * sensor units. Uses Motion Magic for profiled control types and plain position PID for
	 * {@code PIDF}; the {@code NONE} type is a no-op.
	 *
	 * @param position the wanted position
	 */
	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        switch (constants.control.controlConstants.type) {
			case PROFILE, PROFILED_PIDF:
                main.set(TalonSRXControlMode.MotionMagic, position / constants.control.conversionFactor);
				break;

			case PIDF:
                main.set(TalonSRXControlMode.Position, position / constants.control.conversionFactor);
				break;
		}
	}

	/**
	 * Commands the main TalonSRX's onboard closed loop to the given velocity, converted to raw
	 * sensor units. Uses Motion Magic for {@code PROFILED_PIDF} and plain velocity PID for
	 * {@code PIDF}.
	 *
	 * @param velocity the wanted velocity
	 * @throws UnsupportedOperationException if the control type is {@code PROFILE}; the TalonSRX
	 *                                        has no onboard velocity-profile mode
	 */
	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        switch (constants.control.controlConstants.type) {
			case PROFILED_PIDF:
                main.set(TalonSRXControlMode.MotionMagic, velocity / constants.control.conversionFactor);
				break;

			case PIDF:
                main.set(TalonSRXControlMode.Velocity, velocity / constants.control.conversionFactor);
				break;

			case PROFILE:
				throw new UnsupportedOperationException("Velocity profile control not supported on TalonSRX");
		}
	}

	/** Stops the main TalonSRX (and its followers) by commanding zero percent output. */
	@Override
	public void stop() {
		super.stop();
        main.set(TalonSRXControlMode.PercentOutput, 0);
	}

	/** @return the main TalonSRX's selected sensor position, converted by {@link RealControllerConstants.Control#conversionFactor} */
	@Override
	public double getPosition() {
        return main.getSelectedSensorPosition() * constants.control.conversionFactor;
	}

	/** @return the main TalonSRX's selected sensor velocity, converted by {@link RealControllerConstants.Control#conversionFactor} */
	@Override
	public double getVelocity() {
        return main.getSelectedSensorVelocity() * constants.control.conversionFactor;
    }

    /** @return the acceleration computed as the finite-difference change in {@link #getVelocity()} over one 20ms loop */
    @Override
    public double getAcceleration() {
        double acc = (getVelocity() - lastVelocity) / 0.02;
        lastVelocity = getVelocity();
        return acc;
	}

	/** @return the main TalonSRX's applied motor output, between -1 and 1 */
	@Override
	public double getOutput() {
        return main.getMotorOutputPercent();
	}

	/** @return the main TalonSRX's supply current, in amps */
	@Override
	public double getSupplyCurrent() {
        return main.getSupplyCurrent();
	}

	/**
	 * The Phoenix 5 TalonSRX API doesn't separately report stator current, so this returns the
	 * same value as {@link #getSupplyCurrent()}.
	 *
	 * @return the main TalonSRX's supply current, in amps
	 */
	@Override
	public double getStatorCurrent() {
        return main.getSupplyCurrent();
	}

	/**
	 * Overwrites the main TalonSRX's selected sensor position, converted to raw sensor units.
	 *
	 * @param position the position to set the encoder to
	 */
	@Override
	public void setEncoder(double position) {
        main.setSelectedSensorPosition(position / constants.control.conversionFactor);
	}
}
