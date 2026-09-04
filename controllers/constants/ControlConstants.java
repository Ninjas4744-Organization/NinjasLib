package frc.lib.NinjasLib.controllers.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;

/** Proportional Integral Derivative Feedforward, constants for combining PID and Feedforward */
public class ControlConstants implements Cloneable {
    /**
     * Which closed-loop control scheme a {@link Controller} should use, determining which
     * onboard/software control mode {@code setPosition}/{@code setVelocity} dispatch to.
     */
    public enum ControlType {
		/** Plain PID(F) control, running each loop with no motion profiling. */
		PIDF,
		/** PID(F) control wrapped in a trapezoid motion profile (Motion Magic-style). */
		PROFILED_PIDF,
		/** Open-loop trapezoid motion profile, with no PID feedback correction. */
		PROFILE,
		/** Torque-current (FOC) closed-loop control; only supported on TalonFX. */
		TORQUE_CURRENT,
		/** No control configured. */
		NONE
	}

	/**
	 * Main component of PID, Proportional, the bigger it is the faster the PID is, if too big it
	 * overshoots and if too small it doesn't reach the setpoint
	 */
	public double P = 0;

	/**
	 * Integral component of PID, if the PD doesn't reach the setpoint increase this together with
	 * IZone to make have a little push at the end to reach the setpoint
	 */
	public double I = 0;

	/**
	 * Derivative component of PID, increase this to make the PID smoother, it's job is to lower the
	 * speed of the PID if it spikes too much
	 */
	public double D = 0;

	/**
	 * IZone is the error zone to enable the I component, for example, if you work in meters and your
	 * PD doesn't work after error of 0.1m you can set this to 0.15
	 */
	public double IZone = 0;

	/**
	 * Wanted velocity to voltage feedforward
	 */
	public double V = 0;

	/**
	 * Wanted acceleration to voltage feedforward
	 */
	public double A = 0;

	/**
	 * How much voltage to overcome static friction
	 */
	public double S = 0;

	/**
	 * How much voltage to overcome gravity
	 */
	public double G = 0;

	/**
	 * Whether the gravity on the subsystem is like an elevator or line an arm. This will only affect kG if you gave it a number
	 */
    public GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;

	/**
	 * the max velocity the profile should reach
	 */
    public double cruiseVelocity = 0;

	/**
	 * the acceleration in which the profile should increase its velocity until cruise velocity
	 * reached
	 */
    public double acceleration = 0;

	/** the rate of acceleration change in the profile */
    public double jerk = 0;

	/** Which control scheme these constants apply to; set automatically by the {@code createXxx} factory methods below. */
	public ControlType type = ControlType.NONE;

	/**
	 * Builds plain PID constants (control type {@link ControlType#PIDF}, no feedforward terms).
	 *
	 * @param P     proportional gain
	 * @param I     integral gain
	 * @param D     derivative gain
	 * @param IZone error zone below which the integral term is active
	 * @return the new {@link ControlConstants}
	 */
	public static ControlConstants createPID(double P, double I, double D, double IZone) {
		ControlConstants constants = new ControlConstants();
		constants.type = ControlType.PIDF;
		constants.P = P;
		constants.I = I;
		constants.D = D;
		constants.IZone = IZone;
		return constants;
	}

	/**
	 * Builds PID constants with feedforward terms (control type {@link ControlType#PIDF}).
	 *
	 * @param P           proportional gain
	 * @param I           integral gain
	 * @param D           derivative gain
	 * @param IZone       error zone below which the integral term is active
	 * @param V           velocity feedforward
	 * @param A           acceleration feedforward
	 * @param S           static-friction feedforward
	 * @param G           gravity feedforward
	 * @param gravityType whether gravity compensation should behave like an elevator or an arm
	 * @return the new {@link ControlConstants}
	 */
	public static ControlConstants createPIDF(double P, double I, double D, double IZone, double V, double A, double S, double G, GravityTypeValue gravityType) {
		ControlConstants constants = new ControlConstants();
		constants.type = ControlType.PIDF;
		constants.P = P;
		constants.I = I;
		constants.D = D;
		constants.IZone = IZone;
		constants.V = V;
		constants.A = A;
		constants.S = S;
		constants.G = G;
		constants.gravityType = gravityType;
		return constants;
	}

    /**
     * Builds open-loop trapezoid-profile constants (control type {@link ControlType#PROFILE}),
     * with no PID feedback.
     *
     * @param cruiseVelocity the max velocity the profile should reach
     * @param acceleration   the rate at which the profile accelerates up to cruise velocity
     * @param jerk           the rate of acceleration change in the profile
     * @param V              velocity feedforward
     * @param A              acceleration feedforward
     * @param S              static-friction feedforward
     * @param G              gravity feedforward
     * @param gravityType    whether gravity compensation should behave like an elevator or an arm
     * @return the new {@link ControlConstants}
     */
    public static ControlConstants createProfile(double cruiseVelocity, double acceleration, double jerk, double V, double A, double S, double G, GravityTypeValue gravityType) {
		ControlConstants constants = new ControlConstants();
		constants.type = ControlType.PROFILE;
        constants.cruiseVelocity = cruiseVelocity;
        constants.acceleration = acceleration;
        constants.jerk = jerk;
		constants.V = V;
		constants.A = A;
		constants.S = S;
		constants.G = G;
        constants.gravityType = gravityType;
		return constants;
	}

    /**
     * Builds trapezoid-profiled PID constants with feedforward (control type
     * {@link ControlType#PROFILED_PIDF}) - PID feedback correction layered on top of a motion
     * profile, similar to CTRE Motion Magic.
     *
     * @param P              proportional gain
     * @param I              integral gain
     * @param D              derivative gain
     * @param IZone          error zone below which the integral term is active
     * @param cruiseVelocity the max velocity the profile should reach
     * @param acceleration   the rate at which the profile accelerates up to cruise velocity
     * @param jerk           the rate of acceleration change in the profile
     * @param V              velocity feedforward
     * @param A              acceleration feedforward
     * @param S              static-friction feedforward
     * @param G              gravity feedforward
     * @param gravityType    whether gravity compensation should behave like an elevator or an arm
     * @return the new {@link ControlConstants}
     */
    public static ControlConstants createProfiledPIDF(double P, double I, double D, double IZone, double cruiseVelocity, double acceleration, double jerk, double V, double A, double S, double G, GravityTypeValue gravityType) {
		ControlConstants constants = new ControlConstants();
		constants.type = ControlType.PROFILED_PIDF;
		constants.P = P;
		constants.I = I;
		constants.D = D;
		constants.IZone = IZone;
        constants.cruiseVelocity = cruiseVelocity;
        constants.acceleration = acceleration;
        constants.jerk = jerk;
		constants.V = V;
		constants.A = A;
		constants.S = S;
        constants.G = G;
        constants.gravityType = gravityType;
		return constants;
	}

	/**
	 * Builds torque-current FOC constants (control type {@link ControlType#TORQUE_CURRENT}); only
	 * supported on TalonFX.
	 *
	 * @param P proportional gain
	 * @param A acceleration feedforward
	 * @param S static-friction feedforward
	 * @return the new {@link ControlConstants}
	 */
	public static ControlConstants createTorqueCurrent(double P, double A, double S) {
		ControlConstants constants = new ControlConstants();
		constants.type = ControlType.TORQUE_CURRENT;
		constants.P = P;
		constants.A = A;
		constants.S = S;
		return constants;
	}

	/** @return a human-readable dump of every gain, feedforward, and profile constant, for logging/debugging */
	@Override
	public String toString() {
		return String.format("ControlConstants(Type: %s, P: %f, I: %f, D: %f, IZone: %f, V: %f, A: %f, S: %f, G: %f, GravityType: %s, CruiseVelocity: %f, Acceleration: %f, Jerk: %f)",
			type, P, I, D, IZone, V, A, S, G, gravityType.toString(), cruiseVelocity, acceleration, jerk
		);
	}

	/** @return a new {@link ControlConstants} with the same field values as this one */
	@Override
	public ControlConstants clone() {
        ControlConstants clone = new ControlConstants();

		clone.P = this.P;
		clone.I = this.I;
		clone.D = this.D;
		clone.IZone = this.IZone;

		clone.S = this.S;
		clone.V = this.V;
		clone.A = this.A;
		clone.G = this.G;

		clone.gravityType = this.gravityType;

		clone.cruiseVelocity = this.cruiseVelocity;
		clone.acceleration = this.acceleration;
		clone.jerk = this.jerk;

		clone.type = this.type;

		return clone;
	}
}
