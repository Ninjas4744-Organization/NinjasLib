package com.ninjas4744.NinjasLib.Controllers;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants.SmartControlType;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public abstract class NinjasController {
	public enum ControlState {
		PERCENT_OUTPUT,
		POSITION,
		VELOCITY
	}

	protected ControlState _controlState = ControlState.PERCENT_OUTPUT;
	protected MainControllerConstants _constants;
	protected double _goal = 0;

	/**
	 * Creates a new Ninjas controller
	 *
	 * @param constants the constants for the controller
	 */
	public NinjasController(MainControllerConstants constants) {
		_constants = constants;
	}

	/**
	 * Sets percentage output to the controller
	 *
	 * @param percent how much to power the motor between -1 and 1
	 * @see #setPosition(double)
	 * @see #setVelocity(double)
	 * @see #stop()
	 */
	public void setPercent(double percent) {
		_controlState = ControlState.PERCENT_OUTPUT;
	}

	/**
	 * Sets position setpoint to the controller
	 *
	 * @param position the wanted position of the controller according to the encoder
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
	 * @param velocity the wanted velocity of the controller according to the encoder
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
	 * @return the current the motor is taking
	 */
	public abstract double getCurrent();

	/**
	 * Sets the position in the encoder so it thinks it is at that position
	 *
	 * @param position the position to set the encoder to
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
	 * @return goal/setpoint/reference of the controller, the target of PIDF / PID / Motion
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
	public void periodic() {
		if(!_constants.enableLogging)
			return;

		Logger.recordOutput(_constants.subsystemName + "/Position", getPosition());
		Logger.recordOutput(_constants.subsystemName + "/Velocity", getVelocity());
		Logger.recordOutput(_constants.subsystemName + "/Output", getOutput());
		Logger.recordOutput(_constants.subsystemName+"/Current", getCurrent());
		Logger.recordOutput(_constants.subsystemName + "/Goal", getGoal());
		Logger.recordOutput(_constants.subsystemName + "/Control State", _controlState.toString());
		Logger.recordOutput(_constants.subsystemName + "/Control Type", _constants.controlConstants.type == SmartControlType.NONE ? "N/A" : _constants.controlConstants.type.toString());
	}
}
