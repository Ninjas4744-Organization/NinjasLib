package com.ninjas4744.NinjasLib.DataClasses;

public class SimulatedControllerConstants {
	/** Regular controller constants */
	public MainControllerConstants mainControllerConstants = new MainControllerConstants();

	public enum MotorType {
		KRAKEN,
		KRAKEN_FOC,
		FALCON,
		FALCON_FOC,
		NEO,
		NEO550,
		CIM
	}

	/**
	 * The motor that is connected the simulated controller
	 */
	public MotorType motorType = MotorType.KRAKEN;

	/** Torque of the motor in its subsystem(jKgMetersSquared) */
	public double motorTorque = 1;

	/**
	 * Gear ratio between the motor output and the output after the gearbox, bigger values means
	 * bigger reduction(1 / x)
	 */
	public double gearRatio = 1;
}
