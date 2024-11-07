package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Map;

public abstract class NinjasController {
	public enum ControlState {
		PERCENT_OUTPUT,
		PIDF_POSITION,
		PIDF_VELOCITY,
		PID_POSITION,
		PID_VELOCITY,
		FF_POSITION,
		FF_VELOCITY
	}

	private final int shuffleboardEnteriesSize = 3;
	protected ControlState _controlState = ControlState.PERCENT_OUTPUT;
	protected MainControllerConstants _constants;
	protected double _goal = 0;

	/**
	 * Creates a new Ninjas controller
	 *
	 * @param constants - the constants for the controller
	 */
	public NinjasController(MainControllerConstants constants) {
		_constants = constants;

		Shuffleboard.getTab(constants.subsystemName)
				.addDouble("Position", this::getPosition)
				.withWidget("Graph")
				.withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize)
				.withPosition(shuffleboardEnteriesSize, 0)
				.withProperties(Map.of("Automatic bounds", false, "Upper bound", 100, "Lower bound", -100));

		Shuffleboard.getTab(constants.subsystemName)
				.addDouble("Velocity", this::getVelocity)
				.withWidget("Graph")
				.withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize)
				.withPosition(shuffleboardEnteriesSize * 2, 0)
				.withProperties(Map.of("Automatic bounds", false, "Upper bound", 100, "Lower bound", -100));

		Shuffleboard.getTab(constants.subsystemName)
				.addDouble("Output", this::getOutput)
				.withWidget("Graph")
				.withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize)
				.withPosition(0, 0)
				.withProperties(Map.of("Automatic bounds", false, "Upper bound", 1, "Lower bound", -1));

		Shuffleboard.getTab(constants.subsystemName)
				.addDouble("Goal", this::getGoal)
				.withWidget("Number Bar")
				.withSize(shuffleboardEnteriesSize / 2, shuffleboardEnteriesSize)
				.withPosition(shuffleboardEnteriesSize * 3 + 1, 0)
				.withProperties(Map.of("Min", -100, "Max", 100, "Orientation", "VERTICAL"));

		Shuffleboard.getTab(constants.subsystemName)
				.addString("Control State", () -> _controlState.toString())
				.withWidget("Text View")
				.withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize / 2)
				.withPosition(shuffleboardEnteriesSize, shuffleboardEnteriesSize + 1);
	}

	/**
	 * Sets percentage output to the controller
	 *
	 * @param percent - how much to power the motor between -1 and 1
	 * @see #setPosition(double)
	 * @see #setVelocity(double)
	 * @see #stop()
	 */
	public void setPercent(double percent) {
		_controlState = ControlState.PERCENT_OUTPUT;
		_goal = percent;
	}

	/**
	 * Sets position setpoint to the controller
	 *
	 * @param position - the wanted position of the controller according to the encoder
	 * @see #setPercent(double)
	 * @see #setVelocity(double)
	 * @see #stop()
	 */
	public void setPosition(double position) {
		if (_constants.PIDFConstants.kP != 0 || _constants.PIDFConstants.kI != 0 || _constants.PIDFConstants.kD != 0) {
			if (_constants.PIDFConstants.kCruiseVelocity != 0 && _constants.PIDFConstants.kAcceleration != 0)
				_controlState = ControlState.PIDF_POSITION;
			else _controlState = ControlState.PID_POSITION;
		} else {
			if (_constants.PIDFConstants.kCruiseVelocity != 0 && _constants.PIDFConstants.kAcceleration != 0)
				_controlState = ControlState.FF_POSITION;
			else throw new UnsupportedOperationException("PIDF constants were not given for this controller");
		}

		_goal = position;
	}

	/**
	 * Sets velocity setpoint output to the controller
	 *
	 * @param velocity - the wanted velocity of the controller according to the encoder
	 * @see #setPercent(double)
	 * @see #setPosition(double)
	 * @see #stop()
	 */
	public void setVelocity(double velocity) {
		if (_constants.PIDFConstants.kP != 0 || _constants.PIDFConstants.kI != 0 || _constants.PIDFConstants.kD != 0) {
			if (_constants.PIDFConstants.kCruiseVelocity != 0 && _constants.PIDFConstants.kAcceleration != 0)
				_controlState = ControlState.PIDF_VELOCITY;
			else _controlState = ControlState.PID_VELOCITY;
		} else {
			if (_constants.PIDFConstants.kCruiseVelocity != 0 && _constants.PIDFConstants.kAcceleration != 0)
				_controlState = ControlState.FF_VELOCITY;
			else throw new UnsupportedOperationException("PIDF constants were not given for this controller");
		}

		_goal = velocity;
	}

	/**
	 * Stops the controller of all movement
	 *
	 * @see #setPercent(double)
	 * @see #setPosition(double)
	 * @see #setVelocity(double)
	 */
	public void stop() {
		setPercent(0);
	}

	/**
	 * @return the position of the controller
	 */
	public abstract double getPosition();

	/**
	 * @return the velocity of the controller
	 */
	public abstract double getVelocity();

	/**
	 * @return the percent output of the controller
	 */
	public abstract double getOutput();

	/**
	 * Sets the position in the encoder so it thinks it is at that position
	 *
	 * @param position - the position to set the encoder to
	 */
	public abstract void setEncoder(double position);

	/**
	 * Resets the encoder, sets it to the home position
	 *
	 * @see #isHomed
	 */
	public void resetEncoder() {
		setEncoder(_constants.encoderHomePosition);
	}

	/**
	 * @return Whether the subsystem is homed: the encoder is at its home position
	 * @see #resetEncoder
	 */
	public boolean isHomed() {
		return Math.abs(_constants.encoderHomePosition - getPosition()) < _constants.positionGoalTolerance;
	}

	/**
	 * @return the goal/setpoint/reference of the controller, the target of PIDF / PID / Motion
	 *     Magic...
	 */
	public double getGoal() {
		return _goal;
	}

	/**
	 * @return whether or not the controller is at the goal, the target of PIDF / PID / Motion Magic...
	 *     Will return false if not in position or velocity control
	 */
	public boolean atGoal() {
		if (_controlState == ControlState.PIDF_POSITION
				|| _controlState == ControlState.PID_POSITION
				|| _controlState == ControlState.FF_POSITION)
			return Math.abs(getGoal() - getPosition()) < _constants.positionGoalTolerance;
		else if (_controlState == ControlState.PIDF_VELOCITY
				|| _controlState == ControlState.PID_VELOCITY
				|| _controlState == ControlState.FF_VELOCITY)
			return Math.abs(getGoal() - getVelocity()) < _constants.velocityGoalTolerance;

		return false;
	}

	/** Runs controller periodic tasks, run it on the subsystem periodic */
	public void periodic() {}
}
