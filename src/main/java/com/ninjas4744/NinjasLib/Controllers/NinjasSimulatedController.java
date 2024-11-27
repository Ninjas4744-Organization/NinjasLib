package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class NinjasSimulatedController extends NinjasController {
	private DCMotorSim _main;

	private final TrapezoidProfile _profile;
	private final ProfiledPIDController _PIDFController;
	private boolean isCurrentlyPiding = false;
	private double _output = 0;

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

		_main.setInputVoltage(12 * percent);
		_output = percent;
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
		return _output;
	}

	@Override
	public void setEncoder(double position) {
		_main.setState(position * Math.PI * 2 / _constants.encoderConversionFactor, _main.getAngularVelocityRadPerSec());
	}

	@Override
	public void periodic() {
		super.periodic();

		switch (_controlState) {
			case PIDF_POSITION:
				isCurrentlyPiding = true;
				_output = _PIDFController.calculate(getPosition());
				_main.setInputVoltage(12 * _output);
				break;

			case PIDF_VELOCITY:
				isCurrentlyPiding = true;
				_output = _PIDFController.calculate(getVelocity());
				_main.setInputVoltage(12 * _output);
				break;

			case FF_POSITION:
				_output = _profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getGoal(), 0))
					.velocity;
				_main.setInputVoltage(_output * _constants.PIDFConstants.kV);
				break;

			case FF_VELOCITY:
				_output = _profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getPosition(), getVelocity()))
					.velocity;
				_main.setInputVoltage(_output * _constants.PIDFConstants.kV);
				break;
		}

		if (!isCurrentlyPiding) {
			switch (_controlState) {
				case PIDF_POSITION, PIDF_VELOCITY:
					_PIDFController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
					break;
            }
		}
		isCurrentlyPiding = false;

		_main.update(0.02);
	}
}
