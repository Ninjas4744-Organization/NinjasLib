package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSimulated extends SwerveIO {
    private ChassisSpeeds _currentChassisSpeeds = new ChassisSpeeds();
    private final SlewRateLimiter _xAccelerationLimit;
    private final SlewRateLimiter _yAccelerationLimit;
    private final SlewRateLimiter _0AccelerationLimit;

    public SwerveSimulated(SwerveConstants constants){
        super(constants);

        _xAccelerationLimit = new SlewRateLimiter(constants.accelerationLimit);
        _yAccelerationLimit = new SlewRateLimiter(constants.accelerationLimit);
        _0AccelerationLimit = new SlewRateLimiter(constants.rotationAccelerationLimit);
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        _currentChassisSpeeds = new ChassisSpeeds(
            _xAccelerationLimit.calculate(MathUtil.clamp(drive.vxMetersPerSecond, -_constants.speedLimit, _constants.speedLimit)),
            _yAccelerationLimit.calculate(MathUtil.clamp(drive.vyMetersPerSecond, -_constants.speedLimit, _constants.speedLimit)),
            _0AccelerationLimit.calculate(MathUtil.clamp(drive.omegaRadiansPerSecond, -_constants.rotationSpeedLimit, _constants.rotationSpeedLimit))
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
