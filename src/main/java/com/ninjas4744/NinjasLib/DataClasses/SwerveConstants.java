package com.ninjas4744.NinjasLib.DataClasses;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    /** Whether to drive without module velocity PID control */
    public boolean openLoop;

    /** Distance between modules in forward axis */
    public double trackWidth;

    /** Distance between modules in side axis */
    public double wheelBase;

    /** Max speed the swerve could possibly drive */
    public double maxSpeed;

    /** Max speed the swerve could possibly rotate */
    public double maxAngularVelocity;

    /** Percent factor of swerve speed, essentially speed limit, 1 no speed limit, 0 no movement */
    public double speedFactor;

    /** Percent factor of swerve rotational speed, essentially rotational speed limit, 1 no speed limit, 0 no movement */
    public double rotationSpeedFactor;

    /** Swerve max acceleration limit, m/s^2 */
    public double maxAcceleration;

    /** Swerve max rotational acceleration limit, rad/s^2 */
    public double maxRotationAcceleration;

    /** Module Specific Constants */
    public SwerveModuleConstants[] moduleConstants;

    /** Swerve kinematics class used for calculating swerve movement */
    public SwerveDriveKinematics kinematics;

    /** Whether to create shuffleboard tabs for the swerve*/
    public boolean createShuffleBoard;
}
