package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.NinjasLib.dataclasses.*;
import frc.lib.NinjasLib.dataclasses.RealControllerConstants.SimpleControllerConstants;

public class Constants {
    public enum RobotMode {
        /** Running on a real robot */
        REAL,

        /** Running on a simulator */
        SIM,

        /** Replaying from a log file */
        REPLAY
    }

    /* General */
    public static final RobotMode kSimMode = RobotMode.SIM;
    public static final RobotMode kCurrentMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /* Subsystems */
    public static final ControllerConstants kExampleSubsystemControllerConstants = new ControllerConstants();
    static {
        /* Base */
        kExampleSubsystemControllerConstants.real.main.id = 20;
        kExampleSubsystemControllerConstants.real.main.inverted = false;
        kExampleSubsystemControllerConstants.real.currentLimit = 80;
        kExampleSubsystemControllerConstants.real.isBrakeMode = true;

        /* Followers */
        kExampleSubsystemControllerConstants.real.followers = new SimpleControllerConstants[1];
        kExampleSubsystemControllerConstants.real.followers[0] = new SimpleControllerConstants();
        kExampleSubsystemControllerConstants.real.followers[0].id = 21;
        kExampleSubsystemControllerConstants.real.followers[0].inverted = true;

        /* Control */
        kExampleSubsystemControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
        kExampleSubsystemControllerConstants.real.gearRatio = 50;
        kExampleSubsystemControllerConstants.real.conversionFactor = 2 * Math.PI;
        kExampleSubsystemControllerConstants.real.homePosition = Units.degreesToRadians(-60);
        kExampleSubsystemControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(1.5);

        /* Soft Limits */
        kExampleSubsystemControllerConstants.real.maxSoftLimit = Units.degreesToRadians(240);

        /* Hard Limit */
        kExampleSubsystemControllerConstants.real.isLimitSwitch = true;
        kExampleSubsystemControllerConstants.real.limitSwitchID = 2;
        kExampleSubsystemControllerConstants.real.limitSwitchDirection = -1;
        kExampleSubsystemControllerConstants.real.limitSwitchAutoStopReset = true;
        kExampleSubsystemControllerConstants.real.limitSwitchInverted = true;

        /* Simulation */
        kExampleSubsystemControllerConstants.motorType = DCMotor.getKrakenX60(2);
    }
}
