package com.ninjas4744.NinjasLib.DataClasses;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

public class SwerveControllerConstants {
    /** Regular swerve constants */
    public SwerveConstants swerveConstants;

    /** When using swerve controller statemachine, should the driver input be interpreted relative to the field or the robot */
    public boolean driverFieldRelative;

    /** Swerve auto driving PID constants */
    public PIDFConstants swerveDrivePIDConstants;

    /** Swerve auto driving angle PID constants */
    public PIDFConstants swerveAnglePIDConstants;

    /** Swerve auto axis locking PID constants */
    public PIDFConstants swerveAxisPIDConstants;

    /** Profile Constraints(Doesn't affect pathplanner autonomy, only paths created on the fly) */
    public PathConstraints constraints;

    /** Pathplanner autonomy config */
    public HolonomicPathFollowerConfig autonomyConfig;

    /** Swerve drive assist distance threshold. meters */
    public double kDriveAssistThreshold;
}
