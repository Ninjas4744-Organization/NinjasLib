package com.ninjas4744.lib;

import com.ninjas4744.lib.data.MainControllerConstants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

public class NinjasSparkMaxController extends NinjasController {
	private CANSparkMax _main;
	private CANSparkMax[] _followers;

	private TrapezoidProfile _profile;
	private Timer _trapozoidTimer = new Timer();

	public NinjasSparkMaxController(MainControllerConstants constants) {
		super(constants);

		_main = new CANSparkMax(constants.main.id, CANSparkMax.MotorType.kBrushless);

		_main.restoreFactoryDefaults();

		_main.setInverted(constants.main.inverted);
		_main.setSmartCurrentLimit((int) constants.currentLimit);

		_main.getPIDController().setP(constants.PIDFConstants.kP);
		_main.getPIDController().setI(constants.PIDFConstants.kI);
		_main.getPIDController().setD(constants.PIDFConstants.kD);
		_main.getPIDController().setIZone(constants.PIDFConstants.kIZone);
		_main.getPIDController().setFF(constants.PIDFConstants.kF);

		_main.getEncoder().setPositionConversionFactor(constants.encoderConversionFactor);
		_main.getEncoder().setVelocityConversionFactor(constants.encoderConversionFactor / 60);

		_main.burnFlash();

		_followers = new CANSparkMax[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new CANSparkMax(constants.followers[i].id, CANSparkMax.MotorType.kBrushless);
			_followers[i].restoreFactoryDefaults();
			_followers[i].follow(_main, constants.followers[i].inverted);
			_followers[i].burnFlash();
		}

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				constants.PIDFConstants.kCruiseVelocity, constants.PIDFConstants.kAcceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

		_trapozoidTimer.restart();
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		_trapozoidTimer.restart();
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
		return _main.get();
	}

	@Override
	public void setEncoder(double position) {
		_main.getEncoder().setPosition(position);
	}

	@Override
	public boolean atGoal() {
		if (_controlState == ControlState.PIDF_POSITION)
			return Math.abs(_goal - getPosition()) < _constants.positionGoalTolerance;
		else if (_controlState == ControlState.PIDF_VELOCITY)
			return Math.abs(_goal - getVelocity()) < _constants.velocityGoalTolerance;

		return false;
	}

	@Override
	public void periodic() {
		super.periodic();

		if (atGoal()) return;

		switch (_controlState) {
			case PIDF_POSITION:
				_main.getPIDController()
						.setReference(
								_profile.calculate(
												_trapozoidTimer.get(),
												new State(getPosition(), 0),
												new State(getGoal(), 0))
										.position,
								ControlType.kPosition);
				break;

			case PIDF_VELOCITY:
				_main.getPIDController()
						.setReference(
								_profile.calculate(
												_trapozoidTimer.get(),
												new State(0, getVelocity()),
												new State(0, getGoal()))
										.velocity,
								ControlType.kVelocity);
				break;

			case PID_POSITION:
				_main.getPIDController().setReference(getGoal(), ControlType.kPosition);
				break;

			case PID_VELOCITY:
				_main.getPIDController().setReference(getGoal(), ControlType.kVelocity);
				break;

			case FF_POSITION:
				_main.set(_profile.calculate(
										_trapozoidTimer.get(),
										new State(getPosition(), getVelocity()),
										new State(getGoal(), 0))
								.velocity
						/ _constants.PIDFConstants.kMaxVelocity);
				break;

			case FF_VELOCITY:
				_main.set(
						_profile.calculate(_trapozoidTimer.get(), new State(getVelocity(), 0), new State(getGoal(), 0))
										.velocity
								/ _constants.PIDFConstants.kMaxVelocity);
				break;

			default:
				break;
		}
	}
}
