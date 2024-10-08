package com.ninjas4744.lib.Controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.ninjas4744.lib.DataClasses.SimulatedControllerConstants;

public class NinjasSimulatedController extends NinjasController {
	private DCMotorSim _main;

	private final TrapezoidProfile _profile;
	private final Timer _trapozoidTimer = new Timer();
	private final PIDController _pid;

	public NinjasSimulatedController(SimulatedControllerConstants constants) {
		super(constants.mainControllerConstants);

		switch (constants.motorType) {
			case KRAKEN:
				_main = new DCMotorSim(
						DCMotor.getKrakenX60(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
						constants.motorTorque);
				break;
			case FALCON:
				_main = new DCMotorSim(
						DCMotor.getFalcon500(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
						constants.motorTorque);
			case NEO:
				_main = new DCMotorSim(
						DCMotor.getNEO(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
						constants.motorTorque);
			case VORTEX:
				_main = new DCMotorSim(
						DCMotor.getNeoVortex(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
						constants.motorTorque);
			case NEO550:
				_main = new DCMotorSim(
						DCMotor.getNeo550(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
						constants.motorTorque);
		}

		_pid = new PIDController(
				constants.mainControllerConstants.PIDFConstants.kP,
				constants.mainControllerConstants.PIDFConstants.kI,
				constants.mainControllerConstants.PIDFConstants.kD);

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				constants.mainControllerConstants.PIDFConstants.kCruiseVelocity,
				constants.mainControllerConstants.PIDFConstants.kAcceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		for (int i = 0; i < _constants.followers.length; i++) _main.setInput(percent, i);
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
		return _main.getAngularPositionRotations() * _constants.encoderConversionFactor;
	}

	@Override
	public double getVelocity() {
		return _main.getAngularVelocityRPM() / 60 * _constants.encoderConversionFactor;
	}

	@Override
	public double getOutput() {
		return _main.getOutput(0);
	}

	@Override
	public void setEncoder(double position) {
		_main.setState(position * Math.PI * 2, _main.getAngularVelocityRadPerSec());
	}

	@Override
	public void periodic() {
		super.periodic();

		if (atGoal()) return;

		for (int i = 0; i < _constants.followers.length + 1; i++)
			switch (_controlState) {
				case PIDF_POSITION:
					_main.setInput(i, _pid.calculate(getPosition(),
						_profile.calculate(
							_trapozoidTimer.get(),
							new TrapezoidProfile.State(getPosition(), 0),
							new TrapezoidProfile.State(getGoal(), 0))
							.position));
					break;

				case PIDF_VELOCITY:
					_main.setInput(i, _pid.calculate(getVelocity(),
						_profile.calculate(
							_trapozoidTimer.get(),
							new TrapezoidProfile.State(0, getVelocity()),
							new TrapezoidProfile.State(0, getGoal()))
							.velocity));
					break;

				case FF_POSITION:
					_main.setInput(i, _profile.calculate(
						_trapozoidTimer.get(),
						new TrapezoidProfile.State(getPosition(), getVelocity()),
						new TrapezoidProfile.State(getGoal(), 0))
						.velocity
						/ _constants.PIDFConstants.kMaxVelocity);
					break;

				case FF_VELOCITY:
					_main.setInput(i, _profile.calculate(_trapozoidTimer.get(), new TrapezoidProfile.State(getVelocity(), 0), new TrapezoidProfile.State(getGoal(), 0))
						.velocity
						/ _constants.PIDFConstants.kMaxVelocity);
					break;

				default:
					break;
			}
	}
}
