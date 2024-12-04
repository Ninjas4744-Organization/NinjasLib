package com.ninjas4744.NinjasLib.DataClasses;

/** Proportional Integral Derivative Feedforward, constants for combining PID and Feedforward */
public class ControlConstants {
	public enum SmartControlType{
		PID,
		PROFILED_PID,
		PROFILE,
		TORQUE_CURRENT
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
	 * How much voltage to overcome static friction
	 */
	public double S = 0;

	/**
	 * Wanted velocity to voltage feedforward
	 */
	public double V = 0;

	/**
	 * the max velocity the feedforward should reach, this is the velocity the PIDF will be most of
	 * the way
	 */
	public double CruiseVelocity = 0;

	/**
	 * the acceleration in which the feedforward should increase it velocity until cruise velocity
	 * reached
	 */
	public double Acceleration = 0;

	public SmartControlType type = SmartControlType.PID;

	public static ControlConstants createPID(double P, double I, double D, double IZone) {
		ControlConstants constants = new ControlConstants();
		constants.type = SmartControlType.PID;
		constants.P = P;
		constants.I = I;
		constants.D = D;
		constants.IZone = IZone;
		return constants;
	}

	public static ControlConstants createProfile(double CruiseVelocity, double Acceleration, double V, double S) {
		ControlConstants constants = new ControlConstants();
		constants.type = SmartControlType.PROFILE;
		constants.CruiseVelocity = CruiseVelocity;
		constants.Acceleration = Acceleration;
		constants.V = V;
		constants.S = S;
		return constants;
	}

	public static ControlConstants createProfiledPID(double P, double I, double D, double IZone, double CruiseVelocity, double Acceleration, double V, double S) {
		ControlConstants constants = new ControlConstants();
		constants.type = SmartControlType.PROFILED_PID;
		constants.P = P;
		constants.I = I;
		constants.D = D;
		constants.IZone = IZone;
		constants.CruiseVelocity = CruiseVelocity;
		constants.Acceleration = Acceleration;
		constants.V = V;
		constants.S = S;
		return constants;
	}

	public static ControlConstants createTorqueCurrent(double P, double S) {
		ControlConstants constants = new ControlConstants();
		constants.type = SmartControlType.TORQUE_CURRENT;
		constants.P = P;
		constants.S = S;
		return constants;
	}

	@Override
	public String toString() {
		return "P: " + P + " I: " + I + " D: " + D + " IZone: " + IZone + " S: " + S + " V: " + V
				+ " CruiseVelocity: " + CruiseVelocity + " Acceleration: " + Acceleration;
	}
}
