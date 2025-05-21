package frc.lib.NinjasLib.dataclasses;

public class SimulatedControllerConstants {
	/** Regular controller constants */
	public MainControllerConstants mainControllerConstants = new MainControllerConstants();

	public enum MotorType {
		KRAKEN,
		KRAKEN_PRO,
		FALCON,
		FALCON_PRO,
		NEO,
		NEO550,
		CIM
	}

	/**
	 * The motor that is connected the simulated controller
	 */
	public MotorType motorType = MotorType.KRAKEN;
}
