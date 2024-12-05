package com.ninjas4744.NinjasLib.DataClasses;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

public class SwerveControllerConstants {
    /** Regular swerve constants */
    public SwerveConstants swerveConstants;

    /** When using swerve controller statemachine, should the driver input be interpreted relative to the field or the robot */
    public boolean driverFieldRelative;

    /** Swerve auto driving PID constants */
    public ControlConstants drivePIDConstants;

    /** Swerve auto driving angle PID constants */
    public ControlConstants rotationPIDConstants;

    /** Swerve auto axis locking PID constants */
    public ControlConstants axisLockPIDConstants;

    /** Profile Constraints(Doesn't affect pathplanner autonomy, only paths created on the fly) */
    public PathConstraints pathConstraints;

    /** Robot config */
    public RobotConfig robotConfig;

    /** Swerve drive assist distance threshold. meters */
    public double driveAssistThreshold;
}
