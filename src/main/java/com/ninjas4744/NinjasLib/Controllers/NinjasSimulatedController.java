package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class NinjasSimulatedController extends NinjasController {
	private DCMotorSim _main;

	private final TrapezoidProfile _profile;
	private final ProfiledPIDController _PIDFController;
	private boolean isCurrentlyPiding = false;
	private double _output = 0;
	Mechanism2d m_mech2d;
	private final MechanismRoot2d m_mech2dRoot;

	private final MechanismLigament2d m_elevatorMech2d;
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
				constants.mainControllerConstants.PIDFConstants.CruiseVelocity,
				constants.mainControllerConstants.PIDFConstants.Acceleration));

		_PIDFController = new ProfiledPIDController(
			constants.mainControllerConstants.PIDFConstants.P,
			constants.mainControllerConstants.PIDFConstants.I,
			constants.mainControllerConstants.PIDFConstants.D,
			new TrapezoidProfile.Constraints(
				constants.mainControllerConstants.PIDFConstants.CruiseVelocity, constants.mainControllerConstants.PIDFConstants.Acceleration));
				
		m_mech2d = new Mechanism2d(2,constants.mainControllerConstants.maxSoftLimit);

		m_mech2dRoot =  m_mech2d.getRoot("Elevator Root", 10, 0);

		m_elevatorMech2d = m_mech2dRoot.append(
			new MechanismLigament2d("Elevator", constants.mainControllerConstants.maxSoftLimit, 90));
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
				_main.setInputVoltage(_output * _constants.PIDFConstants.V);
				break;

			case FF_VELOCITY:
				_output = _profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getPosition(), getVelocity()))
					.velocity;
				_main.setInputVoltage(_output * _constants.PIDFConstants.V);
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
