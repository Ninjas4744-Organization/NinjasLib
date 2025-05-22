package frc.lib.NinjasLib.dataclasses;

import edu.wpi.first.math.system.plant.DCMotor;

public class ControllerConstants {
	/** Regular controller constants */
	public RealControllerConstants real = new RealControllerConstants();

	/**
	 * Type of motor for simulation control
	 */
	public DCMotor motorType = DCMotor.getKrakenX60(1);
}
