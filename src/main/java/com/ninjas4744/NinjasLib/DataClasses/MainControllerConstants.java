package com.ninjas4744.NinjasLib.DataClasses;

public class MainControllerConstants {
	/** Controller constants for the main controller in the subsystem */
	public ControllerConstants main = new ControllerConstants();

	/** Controller constants for the controllers that follow the main controller in the subsystem */
	public ControllerConstants[] followers = new ControllerConstants[0];

	/** Current limit */
	public double currentLimit = 60;

	/** The name of the subsystem which uses this controller */
	public String subsystemName = "";

	/** Whether to create a shuffleboard tab for this controller's subsystem */
	public boolean createShuffleboard = true;

	/** Control constants */
	public ControlConstants controlConstants = new ControlConstants();

	/* The error which is considered atGoal(). if the PIDF error is smaller than this value it will be considered atGoal() */
	/** The position error which is considered atGoal() */
	public double positionGoalTolerance = 0.05;

	/** The velocity error which is considered atGoal() */
	public double velocityGoalTolerance = 0.05;

	/**
	 * The home position of the system where the limit switch is and is usually 0. when the limit
	 * switch is hit the encoder will reset to this value
	 */
	public double encoderHomePosition = 0;

	/**
	 * The controller works with amount of rotations according to the encoder, the rotations value gets multiplied by this number, choose a number that will result the encoder
	 * to be in meters / degrees
	 */
	public double encoderConversionFactor = 1;

	/** Whether to apply minimum soft limit */
	public boolean isMinSoftLimit = false;

	/** The down soft limit, makes the system unable to move under it */
	public double minSoftLimit = 0;

	/** Whether to apply maximum soft limit */
	public boolean isMaxSoftLimit = false;

	/** The up soft limit, makes the system unable to move above it */
	public double maxSoftLimit = 0;
}
