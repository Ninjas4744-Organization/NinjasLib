package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class NinjasSimulatedController extends NinjasController {
	private DCMotorSim _main;

	private final TrapezoidProfile _profile;
	private ProfiledPIDController _PIDFController;
	private boolean isCurrentlyPiding = false;

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
				break;
			case NEO:
				_main = new DCMotorSim(
						DCMotor.getNEO(constants.mainControllerConstants.followers.length + 1),
						constants.gearRatio,
						constants.motorTorque);
				break;
			case NEO550:
				_main = new DCMotorSim(
						DCMotor.getNeo550(constants.mainControllerConstants.followers.length + 1),
						constants.gearRatio,
						constants.motorTorque);
				break;
			case CIM:
				_main = new DCMotorSim(
					DCMotor.getCIM(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
					constants.motorTorque);
				break;
			case FALCON_FOC:
				_main = new DCMotorSim(
					DCMotor.getFalcon500Foc(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
					constants.motorTorque);
				break;
			case KRAKEN_FOC:
				_main = new DCMotorSim(
					DCMotor.getKrakenX60Foc(constants.mainControllerConstants.followers.length + 1),
					constants.gearRatio,
					constants.motorTorque);
				break;
		}

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				constants.mainControllerConstants.PIDFConstants.kCruiseVelocity,
				constants.mainControllerConstants.PIDFConstants.kAcceleration));

		_PIDFController = new ProfiledPIDController(
			constants.mainControllerConstants.PIDFConstants.kP,
			constants.mainControllerConstants.PIDFConstants.kI,
			constants.mainControllerConstants.PIDFConstants.kD,
			new TrapezoidProfile.Constraints(
				constants.mainControllerConstants.PIDFConstants.kCruiseVelocity, constants.mainControllerConstants.PIDFConstants.kAcceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.setInput(0, percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

		_PIDFController.setGoal(position);
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		_PIDFController.setGoal(velocity);
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

		switch (_controlState) {
			case PIDF_POSITION:
				isCurrentlyPiding = true;
				_main.setInput(0, _PIDFController.calculate(getPosition()));
				break;

			case PIDF_VELOCITY:
				isCurrentlyPiding = true;
				_main.setInput(0, _PIDFController.calculate(getVelocity()));
				break;

			case FF_POSITION:
				_main.setInput(0, _profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getGoal(), 0))
					.velocity
					* _constants.PIDFConstants.kV
					/ 12);
				break;

			case FF_VELOCITY:
				_main.setInput(0, _profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getPosition(), getGoal()))
					.velocity
					* _constants.PIDFConstants.kV
					/ 12);
				break;
		}

		if (!isCurrentlyPiding) {
			switch (_controlState) {
				case PIDF_POSITION:
					_PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
					break;

				case PIDF_VELOCITY:
					_PIDFController.reset(new TrapezoidProfile.State(getVelocity(), 0));
					break;
			}
		}
		isCurrentlyPiding = false;

		_main.update(0.02);
	}
}
