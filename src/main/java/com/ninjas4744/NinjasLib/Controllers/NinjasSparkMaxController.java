package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class NinjasSparkMaxController extends NinjasController {
	private final CANSparkMax _main;
	private final CANSparkMax[] _followers;

	private final TrapezoidProfile _profile;
	private final ProfiledPIDController _PIDFController;
	private boolean isCurrentlyPiding = false;

	public NinjasSparkMaxController(MainControllerConstants constants) {
		super(constants);

		_main = new CANSparkMax(constants.main.id, CANSparkMax.MotorType.kBrushless);

		_main.restoreFactoryDefaults();

		_main.setInverted(constants.main.inverted);
		_main.setSmartCurrentLimit((int) constants.currentLimit);

		_main.getPIDController().setP(constants.PIDFConstants.P);
		_main.getPIDController().setI(constants.PIDFConstants.I);
		_main.getPIDController().setD(constants.PIDFConstants.D);
		_main.getPIDController().setIZone(constants.PIDFConstants.IZone);

		_main.getEncoder().setPositionConversionFactor(constants.encoderConversionFactor);
		_main.getEncoder().setVelocityConversionFactor(constants.encoderConversionFactor / 60);

		_main.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, constants.isMaxSoftLimit);
		_main.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, constants.isMinSoftLimit);
		_main.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) constants.maxSoftLimit);
		_main.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) constants.minSoftLimit);

		_main.burnFlash();

		_followers = new CANSparkMax[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new CANSparkMax(constants.followers[i].id, CANSparkMax.MotorType.kBrushless);
			_followers[i].restoreFactoryDefaults();
			_followers[i].follow(_main, constants.followers[i].inverted);
			_followers[i].burnFlash();
		}

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				constants.PIDFConstants.CruiseVelocity, constants.PIDFConstants.Acceleration));

		_PIDFController = new ProfiledPIDController(
				constants.PIDFConstants.P,
				constants.PIDFConstants.I,
				constants.PIDFConstants.D,
				new TrapezoidProfile.Constraints(
						constants.PIDFConstants.CruiseVelocity, constants.PIDFConstants.Acceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

		if (_controlState == ControlState.PID_POSITION)
			_main.getPIDController().setReference(getGoal(), ControlType.kPosition);

		_PIDFController.setGoal(position);
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		if (_controlState == ControlState.PID_VELOCITY)
			_main.getPIDController().setReference(getGoal(), ControlType.kVelocity);

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

		switch (_controlState) {
			case PIDF_POSITION:
				isCurrentlyPiding = true;
				_main.set(_PIDFController.calculate(getPosition()));
				break;

			case PIDF_VELOCITY:
				isCurrentlyPiding = true;
				_main.set(_PIDFController.calculate(getVelocity()));
				break;

			case FF_POSITION:
				_main.set(_profile.calculate(
								0.02,
								new State(getPosition(), getVelocity()),
								new State(getGoal(), 0))
								.velocity
						* _constants.PIDFConstants.V
						/ 12);
				break;

			case FF_VELOCITY:
				_main.set(_profile.calculate(
								0.02,
								new State(getPosition(), getVelocity()),
								new State(getPosition(), getGoal()))
								.velocity
						* _constants.PIDFConstants.V
						/ 12);
				break;
		}

		if (!isCurrentlyPiding) {
			switch (_controlState) {
				case PIDF_POSITION, PIDF_VELOCITY:
					_PIDFController.reset(new State(getPosition(), getVelocity()));
					break;
			}
		}
		isCurrentlyPiding = false;
	}
}
