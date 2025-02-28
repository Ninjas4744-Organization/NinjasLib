package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SwerveIO {
    private static SwerveIO _instance;
    protected SwerveConstants _constants;
    protected SlewRateLimiter _xAccelerationLimit;
    protected SlewRateLimiter _yAccelerationLimit;
    protected SlewRateLimiter _0AccelerationLimit;

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

    public void setAccelerationLimit(double accelerationLimit){
        _constants.accelerationLimit = accelerationLimit;

        double lastValue = _xAccelerationLimit.lastValue();
        _xAccelerationLimit = new SlewRateLimiter(accelerationLimit);
        _xAccelerationLimit.reset(lastValue);

        lastValue = _yAccelerationLimit.lastValue();
        _yAccelerationLimit = new SlewRateLimiter(accelerationLimit);
        _yAccelerationLimit.reset(lastValue);
    }

    public void setRotationAccelerationLimit(double rotationAccelerationLimit){
        _constants.rotationAccelerationLimit = rotationAccelerationLimit;

        double lastValue = _0AccelerationLimit.lastValue();
        _0AccelerationLimit = new SlewRateLimiter(rotationAccelerationLimit);
        _0AccelerationLimit.reset(lastValue);
    }

    public void setSpeedLimit(double speedLimit){
        _constants.speedLimit = speedLimit;
    }

    public void setRotationSpeedLimit(double rotationSpeedLimit){
        _constants.rotationSpeedLimit = rotationSpeedLimit;
    }

    public void periodic(){

    }
}
