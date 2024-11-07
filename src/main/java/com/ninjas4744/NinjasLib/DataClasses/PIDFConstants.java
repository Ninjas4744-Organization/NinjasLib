package com.ninjas4744.NinjasLib.DataClasses;

/** Proportional Integral Derivative Feedforward, constants for combining PID and Feedforward */
public class PIDFConstants {
	/**
	 * Main component of PID, Proportional, the bigger it is the faster the PID is, if too big it
	 * overshoots and if too small it doesn't reach the setpoint
	 */
	public double kP = 0;

	/**
	 * Integral component of PID, if the PD doesn't reach the setpoint increase this together with
	 * IZone to make have a little push at the end to reach the setpoint
	 */
	public double kI = 0;

	/**
	 * Derivative component of PID, increase this to make the PID smoother, it's job is to lower the
	 * speed of the PID if it spikes too much
	 */
	public double kD = 0;

	/**
	 * IZone is the error zone to enable the I component, for example, if you work in meters and your
	 * PD doesn't work after error of 0.1m you can set this to 0.15
	 */
	public double kIZone = Double.POSITIVE_INFINITY;

	/**
	 * How much voltage to overcome static friction
	 */
	public double kS = 0;

	/**
	 * Wanted velocity to voltage feedforward
	 */
	public double kV = 0;

	/**
	 * the max velocity the feedforward should reach, this is the velocity the PIDF will be most of
	 * the way
	 */
	public double kCruiseVelocity = 0;

	/**
	 * the acceleration in which the feedforward should increase it velocity until cruise velocity
	 * reached
	 */
	public double kAcceleration = 0;

	@Override
	public String toString() {
		return "kP: " + kP + " kI: " + kI + " kD: " + kD + " kIZone: " + kIZone + " kS: " + kS + " kV: " + kV
				+ " kCruiseVelocity: " + kCruiseVelocity + " kAcceleration: " + kAcceleration;
	}
}
