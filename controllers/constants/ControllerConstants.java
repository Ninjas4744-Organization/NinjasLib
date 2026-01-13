package frc.lib.NinjasLib.controllers.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ControllerConstants implements Cloneable {
	/** Regular controller constants */
	public RealControllerConstants real = new RealControllerConstants();

	/**
	 * Type of motor for simulation control
	 */
	public DCMotor motorType = DCMotor.getKrakenX60(1);

	@Override
	public ControllerConstants clone() {
		ControllerConstants clone = new ControllerConstants();

		clone.real = new RealControllerConstants();
		clone.real.base.main = new RealControllerConstants.Base.SimpleControllerConstants();
		clone.real.base.main.id = this.real.base.main.id;
		clone.real.base.main.inverted = this.real.base.main.inverted;

		clone.real.base.isBrakeMode = this.real.base.isBrakeMode;

		clone.real.base.followers = new RealControllerConstants.Base.SimpleControllerConstants[this.real.base.followers.length];
		for (int i = 0; i < this.real.base.followers.length; i++) {
			RealControllerConstants.Base.SimpleControllerConstants followerClone =
				new RealControllerConstants.Base.SimpleControllerConstants();
			followerClone.id = this.real.base.followers[i].id;
			followerClone.inverted = this.real.base.followers[i].inverted;
			clone.real.base.followers[i] = followerClone;
		}

		clone.real.base.currentLimit = this.real.base.currentLimit;
		clone.real.base.CANBus = this.real.base.CANBus;

		clone.real.control.gearRatio = this.real.control.gearRatio;
		clone.real.control.conversionFactor = this.real.control.conversionFactor;
		clone.real.control.positionGoalTolerance = this.real.control.positionGoalTolerance;
		clone.real.control.velocityGoalTolerance = this.real.control.velocityGoalTolerance;
		clone.real.control.controlConstants = this.real.control.controlConstants.clone();

		clone.real.softLimits.min = this.real.softLimits.min;
		clone.real.softLimits.max = this.real.softLimits.max;

		clone.real.hardLimit.homePosition = this.real.hardLimit.homePosition;
		clone.real.hardLimit.enable = this.real.hardLimit.enable;
		clone.real.hardLimit.isVirtual = this.real.hardLimit.isVirtual;
		clone.real.hardLimit.virtualStallThreshold = this.real.hardLimit.virtualStallThreshold;
		clone.real.hardLimit.id = this.real.hardLimit.id;
		clone.real.hardLimit.inverted = this.real.hardLimit.inverted;
		clone.real.hardLimit.direction = this.real.hardLimit.direction;
		clone.real.hardLimit.autoStopReset = this.real.hardLimit.autoStopReset;

		clone.motorType = this.motorType;

		return clone;
	}
}
