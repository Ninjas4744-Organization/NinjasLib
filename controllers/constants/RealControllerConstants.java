package frc.lib.NinjasLib.controllers.constants;

public class RealControllerConstants {
    /**
     * Controller constants for the main controller in the subsystem
     */
    public SimpleControllerConstants main = new SimpleControllerConstants();

    /**
     * Whether the neutral mode of the controller should be brake or coast(in brake mode setting motor to 0 makes a sudden stop, in coast mode setting motor to 0 makes it free and slowly climb down to zero due to friction).
     */
    public boolean isBrakeMode = true;

    /**
     * Controller constants for the controllers that follow the main controller in the subsystem.
     */
    public SimpleControllerConstants[] followers = new SimpleControllerConstants[0];

    /**
     * Current limit
     */
    public double currentLimit = 60;

    /**
     * Control constants
     */
    public ControlConstants controlConstants = new ControlConstants();

    /** The error which is considered atGoal(). if the error from the position goal is smaller than this value it will be considered atGoal(). */
    public double positionGoalTolerance = 0.05;

    /** The error which is considered atGoal(). if the error from the velocity goal is smaller than this value it will be considered atGoal(). */
    public double velocityGoalTolerance = 0.05;

    /**
     * The home position of the subsystem where the limit switch is and is usually 0. when the limit
     * switch is hit the encoder will reset to this value.
     */
    public double homePosition = 0;

    /** The gear ratio between the motor and the subsystem including gears, pullies, gearboxes, etc. */
    public double gearRatio = 1;

    /**
     * Conversion between rotations of subsystem to whatever.
     * The calculation is: (rotations of motor) / gearRatio * conversionFactor.
     * So for example if I have an arm with gear ratio of 10, and I want it to be in degrees I would put 360.
     */
    public double conversionFactor = 1;

    /** The down soft limit, makes the system unable to move under it */
    public double minSoftLimit = Double.MIN_VALUE;

    /** The up soft limit, makes the system unable to move above it */
    public double maxSoftLimit = Double.MAX_VALUE;

    /** Whether there is a limit switch related to the subsystem. */
    public boolean isLimitSwitch = false;

    /** Whether to use a virtual limit switch (according to the current the motor takes) instead of a real one. */
    public boolean isVirtualLimit = false;

    /** How much normalized current (current / voltage) is needed to activate the virtual limit to behave like a real limit switch */
    public double virtualLimitStallThreshold = 30 / 12.0;

    /** ID of limit switch used in the subsystem. */
    public int limitSwitchID = 0;

    /** Whether the limit switch is inverted. */
    public boolean limitSwitchInverted = false;

    /** the direction of movement in which the limit will be clicked, for example if an elevator goes down when given minus as output and the limit switch is at the bottom then this value should be -1. */
    public int limitSwitchDirection = -1;

    /** Whether to automatically stop the motor and reset the encoder when limit is clicked. */
    public boolean limitSwitchAutoStopReset = true;

    public static class SimpleControllerConstants {
        /**
         * The ID of the controller, chosen in the device's configuration software like Phoenix Tuner X or Rev
         * Hardware Client
         */
        public int id;

        /**
         * Whether to invert the output of this controller. If this controller is a follower it
         * will invert the main controller's output so if the main controller is inverted and this
         * follower is inverted it will be inverted twice so not inverted.
         */
        public boolean inverted = false;
    }
}
