package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants.SmartControlType;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Map;

public abstract class NinjasController {
	public enum ControlState {
		PERCENT_OUTPUT,
		POSITION,
		VELOCITY
	}

	private final int shuffleboardEnteriesSize = 3;
	protected ControlState _controlState = ControlState.PERCENT_OUTPUT;
	protected SmartControlType _smartControlType = SmartControlType.PID;
	protected MainControllerConstants _constants;
	protected double _goal = 0;

	/**
	 * Creates a new Ninjas controller
	 *
	 * @param constants - the constants for the controller
	 */
	public NinjasController(MainControllerConstants constants) {
		_constants = constants;

		try {
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
		} catch (Exception e) {
			System.err.println("Shuffleboard error occurred while creating " + constants.subsystemName + " controller.");
			System.err.println("Make sure if this controller's subsystem name is unique.");
		}
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
		_controlState = ControlState.POSITION;
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
		_controlState = ControlState.VELOCITY;
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
		if (_controlState == ControlState.POSITION)
			return Math.abs(getGoal() - getPosition()) < _constants.positionGoalTolerance;
		else if (_controlState == ControlState.VELOCITY)
			return Math.abs(getGoal() - getVelocity()) < _constants.velocityGoalTolerance;

		return false;
	}

	/** Runs controller periodic tasks, run it on the subsystem periodic */
	public void periodic() {}
}
