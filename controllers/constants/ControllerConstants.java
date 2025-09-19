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
		clone.real.main = new RealControllerConstants.SimpleControllerConstants();
		clone.real.main.id = this.real.main.id;
		clone.real.main.inverted = this.real.main.inverted;

		clone.real.isBrakeMode = this.real.isBrakeMode;

		clone.real.followers = new RealControllerConstants.SimpleControllerConstants[this.real.followers.length];
		for (int i = 0; i < this.real.followers.length; i++) {
			RealControllerConstants.SimpleControllerConstants followerClone =
				new RealControllerConstants.SimpleControllerConstants();
			followerClone.id = this.real.followers[i].id;
			followerClone.inverted = this.real.followers[i].inverted;
			clone.real.followers[i] = followerClone;
		}

		clone.real.currentLimit = this.real.currentLimit;
		clone.real.positionGoalTolerance = this.real.positionGoalTolerance;
		clone.real.velocityGoalTolerance = this.real.velocityGoalTolerance;
		clone.real.homePosition = this.real.homePosition;
		clone.real.gearRatio = this.real.gearRatio;
		clone.real.conversionFactor = this.real.conversionFactor;
		clone.real.minSoftLimit = this.real.minSoftLimit;
		clone.real.maxSoftLimit = this.real.maxSoftLimit;
		clone.real.isLimitSwitch = this.real.isLimitSwitch;
		clone.real.isVirtualLimit = this.real.isVirtualLimit;
		clone.real.virtualLimitStallThreshold = this.real.virtualLimitStallThreshold;
		clone.real.limitSwitchID = this.real.limitSwitchID;
		clone.real.limitSwitchInverted = this.real.limitSwitchInverted;
		clone.real.limitSwitchDirection = this.real.limitSwitchDirection;
		clone.real.limitSwitchAutoStopReset = this.real.limitSwitchAutoStopReset;
		clone.real.CANBus = this.real.CANBus;

		clone.real.controlConstants = this.real.controlConstants.clone();

		clone.motorType = this.motorType;

		return clone;
	}
}
