package frc.lib.NinjasLib.controllers;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.NinjasLib.dataclasses.ControlConstants.SmartControlType;
import frc.lib.NinjasLib.dataclasses.RealControllerConstants;

public class SparkMaxController extends Controller {
	private final SparkMax _main;
	private final SparkMax[] _followers;

	private final TrapezoidProfile _profile;
	private final ProfiledPIDController _PIDFController;
	private boolean isCurrentlyPiding = false;

    public SparkMaxController(RealControllerConstants constants) {
		super(constants);

		_main = new SparkMax(constants.main.id, SparkMax.MotorType.kBrushless);

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(constants.main.inverted);
		config.smartCurrentLimit((int)constants.currentLimit);

		config.softLimit.forwardSoftLimit(constants.maxSoftLimit)
		.reverseSoftLimit(constants.minSoftLimit)
            .forwardSoftLimitEnabled(constants.maxSoftLimit != Double.MAX_VALUE)
            .reverseSoftLimitEnabled(constants.minSoftLimit != Double.MIN_VALUE);

		config.closedLoop.pid(constants.controlConstants.P, constants.controlConstants.I, constants.controlConstants.D);

        config.encoder.positionConversionFactor(constants.conversionFactor)
            .velocityConversionFactor(constants.conversionFactor / 60);

		_main.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		_followers = new SparkMax[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new SparkMax(constants.followers[i].id, SparkMax.MotorType.kBrushless);

			SparkMaxConfig followerConfig = new SparkMaxConfig();
			followerConfig.follow(_main, constants.followers[i].inverted);
			_followers[i].configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
		}

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            constants.controlConstants.cruiseVelocity, constants.controlConstants.acceleration));

		_PIDFController = new ProfiledPIDController(
				constants.controlConstants.P,
				constants.controlConstants.I,
				constants.controlConstants.D,
				new TrapezoidProfile.Constraints(
                    constants.controlConstants.cruiseVelocity, constants.controlConstants.acceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        if (constants.controlConstants.type == SmartControlType.PID)
			_main.getClosedLoopController().setReference(getGoal(), SparkBase.ControlType.kPosition);

		_PIDFController.setGoal(position);
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        if (constants.controlConstants.type == SmartControlType.PID)
			_main.getClosedLoopController().setReference(getGoal(), SparkBase.ControlType.kVelocity);

		_PIDFController.setGoal(velocity);
	}

	@Override
	public void stop() {
		_main.stopMotor();
	}

	@Override
	public double getPosition() {
		return _main.getEncoder().getPosition();
	}

	@Override
	public double getVelocity() {
		return _main.getEncoder().getVelocity();
	}

	@Override
	public double getOutput() {
		return _main.getBusVoltage() * _main.getAppliedOutput() / 12;
	}

	@Override
	public double getCurrent() {
		return _main.getOutputCurrent();
	}

	@Override
	public void setEncoder(double position) {
		_main.getEncoder().setPosition(position);
	}

	@Override
	public void periodic() {
        switch (constants.controlConstants.type) {
			case PROFILED_PID:
				isCurrentlyPiding = true;

                if (controlState == ControlState.POSITION)
					_main.set(_PIDFController.calculate(getPosition()) / 12);
                else if (controlState == ControlState.VELOCITY)
					_main.set(_PIDFController.calculate(getVelocity()) / 12);
				break;

			case PROFILE:
                if (controlState == ControlState.POSITION)
					_main.set(_profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getGoal(), 0))
						.velocity / 12);
                else if (controlState == ControlState.VELOCITY)
					_main.set(_profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getPosition(), getGoal()))
						.velocity / 12);
				break;
		}

        if (!isCurrentlyPiding && controlState != ControlState.PERCENT_OUTPUT)
			_PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
		isCurrentlyPiding = false;

		super.periodic();
	}
}
