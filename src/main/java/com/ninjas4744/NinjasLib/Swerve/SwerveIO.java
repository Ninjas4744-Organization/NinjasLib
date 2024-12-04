package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SwerveIO {
    private static SwerveIO _instance;
    protected SwerveConstants _constants;

    /** Returns the swerve instance, simulated/real depends on if the code is simulated/real. */
    public static SwerveIO getInstance() {
        if (_instance == null)
            throw new RuntimeException("SwerveIO constants not given. Initialize SwerveIO by setConstants(SwerveConstants) first.");
        return _instance;
    }

    public static SwerveIO setConstants(SwerveConstants constants){
        if (!RobotStateIO.isSimulated()) 
            _instance = new Swerve(constants);
        else
            _instance = new SwerveSimulated(constants);

        return _instance;
    }

    protected SwerveIO(SwerveConstants constants){
        _constants = constants;
    }

    /**
     * Drives the robot
     * @param drive Chassis speeds to drive according to
     * @param fieldRelative Whether to move to robot relative to the field or the robot
     */
    public abstract void drive(ChassisSpeeds drive, boolean fieldRelative);

    /**
     * Get the velocity and angular velocity of the swerve
     * @param fieldRelative Whether to return the velocity relative to the field or the robot
     * @return The velocities
     */
    public abstract ChassisSpeeds getChassisSpeeds(boolean fieldRelative);

    /**
     * Stops the swerve
     */
    public void stop(){
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    /**
     * Convert percent chassis speeds to m/s chassis speeds
     * @param percent the percent chassis speeds to convert
     * @return the m/s chassis speeds to give the swerve
     */
    public ChassisSpeeds fromPercent(ChassisSpeeds percent) {
        return new ChassisSpeeds(
            percent.vxMetersPerSecond * _constants.maxSpeed,
            percent.vyMetersPerSecond * _constants.maxSpeed,
            percent.omegaRadiansPerSecond * _constants.maxAngularVelocity
        );
    }

    public void periodic(){

    }
}
