package frc.lib.NinjasLib.controllers.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import java.util.function.Supplier;

public class ControllerConstants implements Cloneable {
	/** Regular controller constants */
	public RealControllerConstants real = new RealControllerConstants();

	/**
	 * Supplier for the simulated system. Can be any LinearSystemSim subclass -
	 * ElevatorSim, SingleJointedArmSim, DCMotorSim, FlywheelSim, or a custom one.
	 */
	public Supplier<LinearSystemSim<?, ?, ?>> simSystem =
		() -> new ElevatorSim(DCMotor.getKrakenX60Foc(2), real.control.gearRatio, 10, 0.03, 0, 1, true, 0);

	public ControllerConstants withBase(RealControllerConstants.Base base) {
		this.real = this.real.withBase(base);
		return this;
	}

	public ControllerConstants withControl(RealControllerConstants.Control control) {
		this.real = this.real.withControl(control);
		return this;
	}

	public ControllerConstants withSoftLimits(RealControllerConstants.SoftLimits softLimits) {
		this.real = this.real.withSoftLimits(softLimits);
		return this;
	}

	public ControllerConstants withHardLimits(RealControllerConstants.HardLimits.HardLimit[] hardLimits) {
		this.real = this.real.withHardLimits(hardLimits);
		return this;
	}

	public ControllerConstants withCANCoder(RealControllerConstants.CANCoder canCoder) {
		this.real = this.real.withCANCoder(canCoder);
		return this;
	}

	public ControllerConstants withSim(Supplier<LinearSystemSim<?, ?, ?>> simSystem) {
		this.simSystem = simSystem;
		return this;
	}

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
			clone.real.hardLimits.limits[i].minPos = this.real.hardLimits.limits[i].minPos;
			clone.real.hardLimits.limits[i].maxPos = this.real.hardLimits.limits[i].maxPos;
			clone.real.hardLimits.limits[i].frames = this.real.hardLimits.limits[i].frames;
			clone.real.hardLimits.limits[i].inverted = this.real.hardLimits.limits[i].inverted;
			clone.real.hardLimits.limits[i].direction = this.real.hardLimits.limits[i].direction;
			clone.real.hardLimits.limits[i].enableLimitTriggerMethod = this.real.hardLimits.limits[i].enableLimitTriggerMethod;
			clone.real.hardLimits.limits[i].limitTriggerMethod = this.real.hardLimits.limits[i].limitTriggerMethod;
			clone.real.hardLimits.limits[i].homePosition = this.real.hardLimits.limits[i].homePosition;
		}

		clone.real.canCoder.enable = this.real.canCoder.enable;
		clone.real.canCoder.id = this.real.canCoder.id;
		clone.real.canCoder.config = new com.ctre.phoenix6.configs.CANcoderConfiguration();
		clone.real.canCoder.config.MagnetSensor.MagnetOffset = this.real.canCoder.config.MagnetSensor.MagnetOffset;
		clone.real.canCoder.config.MagnetSensor.SensorDirection = this.real.canCoder.config.MagnetSensor.SensorDirection;
		clone.real.canCoder.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = this.real.canCoder.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint;
		clone.real.canCoder.mode = this.real.canCoder.mode;

		clone.simSystem = this.simSystem;

		return clone;
	}
}