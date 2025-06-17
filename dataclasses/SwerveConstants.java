package frc.lib.NinjasLib.dataclasses;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveConstants {
    /** Whether to drive without module velocity PID control */
    public boolean openLoop;

    /** Distance between modules in forward axis */
    public double trackWidth;

    /** Distance between modules in side axis */
    public double wheelBase;

    /** Max speed the swerve could possibly drive, the real thing if you want a speed limit do it in the speedLimit, in m/s */
    public double maxSpeed;

    /** Max speed the swerve could possibly rotate, the real thing if you want a speed limit do it in the rotationSpeedLimit, in rad/s */
    public double maxAngularVelocity;

    /** Max acceleration the swerve could possibly drive, in m/s^2 */
    public double maxAcceleration;

    /** Max acceleration the swerve could possibly change movement direction (needs to be calibrated), in m/s^2 */
    public double maxSkidAcceleration;

    /** Swerve speed limit, the swerve can't drive faster than this number +-, in m/s */
    public double speedLimit;

    /** Swerve rotation speed limit, the swerve can't rotate faster than this number +-, in rad/s */
    public double rotationSpeedLimit;

    /** Swerve max acceleration limit, m/s^2 */
    public double accelerationLimit;

    /** Swerve max rotational acceleration limit, rad/s^2 */
    public double rotationAccelerationLimit;

    /** Module Specific Constants */
    public SwerveModuleConstants[] moduleConstants;

    /** Swerve kinematics class used for calculating swerve movement */
    public SwerveDriveKinematics kinematics;

    /**
     * Robot config
     */
    public RobotConfig robotConfig;

    /* Simulation */
    /**
     * Width of bumper from side to side of robot, meters
     */
    public double bumperWidth;

    /**
     * Length of bumper from back to forward of robot, meters
     */
    public double bumperLength;

    public DCMotor driveMotorType;

    public DCMotor steerMotorType;
}
