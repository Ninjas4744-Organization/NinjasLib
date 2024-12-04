package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants.SmartControlType;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class NinjasSparkMaxController extends NinjasController {
	private final SparkMax _main;
	private final SparkMax[] _followers;

	private final TrapezoidProfile _profile;
	private final ProfiledPIDController _PIDFController;
	private boolean isCurrentlyPiding = false;

	public NinjasSparkMaxController(MainControllerConstants constants) {
		super(constants);

		_main = new SparkMax(constants.main.id, SparkMax.MotorType.kBrushless);

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(constants.main.inverted);
		config.smartCurrentLimit((int)constants.currentLimit);

		config.softLimit.forwardSoftLimit(constants.maxSoftLimit)
		.reverseSoftLimit(constants.minSoftLimit)
		.forwardSoftLimitEnabled(constants.isMaxSoftLimit)
		.reverseSoftLimitEnabled(constants.isMinSoftLimit);

		config.closedLoop.pid(constants.controlConstants.kP, constants.controlConstants.kI, constants.controlConstants.kD);

		config.encoder.positionConversionFactor(constants.encoderConversionFactor)
		.velocityConversionFactor(constants.encoderConversionFactor / 60);

		_main.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		_followers = new SparkMax[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new SparkMax(constants.followers[i].id, SparkMax.MotorType.kBrushless);

			SparkMaxConfig followerConfig = new SparkMaxConfig();
			followerConfig.follow(_main, constants.followers[i].inverted);
			_followers[i].configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
		}

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				constants.controlConstants.kCruiseVelocity, constants.controlConstants.kAcceleration));

		_PIDFController = new ProfiledPIDController(
				constants.controlConstants.kP,
				constants.controlConstants.kI,
				constants.controlConstants.kD,
				new TrapezoidProfile.Constraints(
						constants.controlConstants.kCruiseVelocity, constants.controlConstants.kAcceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

		if (_constants.controlConstants.type == SmartControlType.PID)
			_main.getClosedLoopController().setReference(getGoal(), SparkBase.ControlType.kPosition);

		_PIDFController.setGoal(position);
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		if (_constants.controlConstants.type == SmartControlType.PID)
			_main.getClosedLoopController().setReference(getGoal(), SparkBase.ControlType.kVelocity);

		_PIDFController.setGoal(velocity);
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
	public void setEncoder(double position) {
		_main.getEncoder().setPosition(position);
	}

	@Override
	public void periodic() {
		super.periodic();

		switch (_constants.controlConstants.type) {
			case PROFILED_PID:
				isCurrentlyPiding = true;

				if(_controlState == ControlState.POSITION)
					_main.set(_PIDFController.calculate(getPosition()));
				else if(_controlState == ControlState.VELOCITY)
					_main.set(_PIDFController.calculate(getVelocity()));
				break;

			case PROFILE:
				if(_controlState == ControlState.POSITION)
					_main.set(_profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getGoal(), 0))
					.velocity);
				else if(_controlState == ControlState.VELOCITY)
					_main.set(_profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getPosition(), getGoal()))
					.velocity);
				break;
		}

		if (!isCurrentlyPiding && _controlState != ControlState.PERCENT_OUTPUT)
			_PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
		isCurrentlyPiding = false;
	}
}
