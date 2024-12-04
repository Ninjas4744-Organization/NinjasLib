package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSimulated extends SwerveIO {
    private ChassisSpeeds _currentChassisSpeeds = new ChassisSpeeds();
    private final SlewRateLimiter _xAcceleration;
    private final SlewRateLimiter _yAcceleration;
    private final SlewRateLimiter _0Acceleration;

    public SwerveSimulated(SwerveConstants constants){
        super(constants);

        _xAcceleration = new SlewRateLimiter(constants.simulationAcceleration);
        _yAcceleration = new SlewRateLimiter(constants.simulationAcceleration);
        _0Acceleration = new SlewRateLimiter(constants.simulationAngleAcceleration);
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        _currentChassisSpeeds = drive;
        if(!fieldRelative)
            _currentChassisSpeeds.toFieldRelativeSpeeds(RobotStateWithSwerve.getInstance().getGyroYaw());

        RobotStateWithSwerve.getInstance().setRobotPose(new Pose2d(
            RobotStateWithSwerve.getInstance().getRobotPose().getX()
                + _xAcceleration.calculate(_currentChassisSpeeds.vxMetersPerSecond)
                * 0.02,
            RobotStateWithSwerve.getInstance().getRobotPose().getY()
                + _yAcceleration.calculate(_currentChassisSpeeds.vyMetersPerSecond)
                * 0.02,
            RobotStateWithSwerve.getInstance().getRobotPose()
                .getRotation()
                .plus(Rotation2d.fromRadians(_0Acceleration.calculate(_currentChassisSpeeds.omegaRadiansPerSecond)
                    * 0.02))));
    }

    @Override
    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
        if(fieldRelative)
            return _currentChassisSpeeds;
        else{
            ChassisSpeeds speeds = new ChassisSpeeds(_currentChassisSpeeds.vxMetersPerSecond, _currentChassisSpeeds.vyMetersPerSecond, _currentChassisSpeeds.omegaRadiansPerSecond);
            speeds.toRobotRelativeSpeeds(RobotStateWithSwerve.getInstance().getGyroYaw());
            return speeds;
        }
    }
}
