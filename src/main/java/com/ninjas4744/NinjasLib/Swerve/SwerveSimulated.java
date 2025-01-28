package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
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

        _xAcceleration = new SlewRateLimiter(constants.maxAcceleration);
        _yAcceleration = new SlewRateLimiter(constants.maxAcceleration);
        _0Acceleration = new SlewRateLimiter(constants.maxRotationAcceleration);
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        _currentChassisSpeeds = new ChassisSpeeds(
            _xAcceleration.calculate(drive.vxMetersPerSecond * _constants.speedFactor),
            _yAcceleration.calculate(drive.vyMetersPerSecond * _constants.speedFactor),
            _0Acceleration.calculate(drive.omegaRadiansPerSecond * _constants.rotationSpeedFactor)
        );
        _currentChassisSpeeds = fieldRelative ? _currentChassisSpeeds : ChassisSpeeds.fromRobotRelativeSpeeds(_currentChassisSpeeds, RobotStateWithSwerve.getInstance().getGyroYaw());

        RobotStateWithSwerve.getInstance().setRobotPose(new Pose2d(
            RobotStateWithSwerve.getInstance().getRobotPose().getX()
                + _currentChassisSpeeds.vxMetersPerSecond * 0.02,
            RobotStateWithSwerve.getInstance().getRobotPose().getY()
                + _currentChassisSpeeds.vyMetersPerSecond * 0.02,
            RobotStateWithSwerve.getInstance().getRobotPose().getRotation()
                .plus(Rotation2d.fromRadians(_currentChassisSpeeds.omegaRadiansPerSecond * 0.02))));
    }

    @Override
    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
        return fieldRelative ? _currentChassisSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(_currentChassisSpeeds, RobotStateWithSwerve.getInstance().getGyroYaw());
    }
}
