package com.ninjas4744.lib.data;

public class SimulatedControllerConstants {
	/** Regular controller constants */
	public MainControllerConstants mainControllerConstants = new MainControllerConstants();

	public enum MotorType {
		KRAKEN,
		FALCON,
		VORTEX,
		NEO,
		NEO550
	}

	public MotorType motorType = MotorType.KRAKEN;
	/** Torque of the motor in its subsystem(jKgMetersSquared) */
	public double motorTorque = 1;
}
