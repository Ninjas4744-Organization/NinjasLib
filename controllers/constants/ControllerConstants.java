package frc.lib.NinjasLib.controllers.constants;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class ControllerConstants implements Cloneable {
	/** Regular controller constants */
	public RealControllerConstants real = new RealControllerConstants();

	/** Type of motor for simulation control */
	public DCMotor simMotor = DCMotor.getKrakenX60(real.base.followers.length + 1);

	/** Type of system for simulation control */
	public LinearSystem<N2, N1, N2> simSystem = LinearSystemId.createElevatorSystem(simMotor, 6, 0.03, real.control.gearRatio);

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

		clone.real.base.statorCurrentLimit = this.real.base.statorCurrentLimit;
		clone.real.base.supplyCurrentLimit = this.real.base.supplyCurrentLimit;
		clone.real.base.CANBus = this.real.base.CANBus;

		clone.real.control.gearRatio = this.real.control.gearRatio;
		clone.real.control.conversionFactor = this.real.control.conversionFactor;
		clone.real.control.positionGoalTolerance = this.real.control.positionGoalTolerance;
		clone.real.control.velocityGoalTolerance = this.real.control.velocityGoalTolerance;
		clone.real.control.controlConstants = this.real.control.controlConstants.clone();
		clone.real.control.enableFOC = this.real.control.enableFOC;

		clone.real.softLimits.min = this.real.softLimits.min;
		clone.real.softLimits.max = this.real.softLimits.max;

		clone.real.hardLimits.limits = new RealControllerConstants.HardLimits.HardLimit[this.real.hardLimits.limits.length];
		for (int i = 0; i < this.real.hardLimits.limits.length; i++) {
			clone.real.hardLimits.limits[i] = new RealControllerConstants.HardLimits.HardLimit();
			clone.real.hardLimits.limits[i].id = this.real.hardLimits.limits[i].id;
			clone.real.hardLimits.limits[i].isVirtual = this.real.hardLimits.limits[i].isVirtual;
			clone.real.hardLimits.limits[i].virtualStallThreshold = this.real.hardLimits.limits[i].virtualStallThreshold;
			clone.real.hardLimits.limits[i].virtualMinPos = this.real.hardLimits.limits[i].virtualMinPos;
			clone.real.hardLimits.limits[i].virtualMaxPos = this.real.hardLimits.limits[i].virtualMaxPos;
			clone.real.hardLimits.limits[i].virtualFrames = this.real.hardLimits.limits[i].virtualFrames;
			clone.real.hardLimits.limits[i].inverted = this.real.hardLimits.limits[i].inverted;
			clone.real.hardLimits.limits[i].direction = this.real.hardLimits.limits[i].direction;
			clone.real.hardLimits.limits[i].autoStopReset = this.real.hardLimits.limits[i].autoStopReset;
			clone.real.hardLimits.limits[i].homePosition = this.real.hardLimits.limits[i].homePosition;
		}

		clone.real.canCoder.enable = this.real.canCoder.enable;
		clone.real.canCoder.id = this.real.canCoder.id;
		clone.real.canCoder.config = new com.ctre.phoenix6.configs.CANcoderConfiguration();
		clone.real.canCoder.config.MagnetSensor.MagnetOffset = this.real.canCoder.config.MagnetSensor.MagnetOffset;
		clone.real.canCoder.config.MagnetSensor.SensorDirection = this.real.canCoder.config.MagnetSensor.SensorDirection;
		clone.real.canCoder.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = this.real.canCoder.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint;
		clone.real.canCoder.mode = this.real.canCoder.mode;

		clone.simMotor = this.simMotor;
		clone.simSystem = this.simSystem;

		return clone;
	}
}
