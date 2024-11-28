package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class SwerveSimulated extends SwerveIO {
    private ChassisSpeeds _currentChassisSpeeds = new ChassisSpeeds();
    private final SlewRateLimiter _xAcceleration;
    private final SlewRateLimiter _yAcceleration;
    private final SlewRateLimiter _0Acceleration;

    public SwerveSimulated(SwerveConstants constants){
        super(constants);

        _xAcceleration = new SlewRateLimiter(constants.Simulation.kAcceleration);
        _yAcceleration = new SlewRateLimiter(constants.Simulation.kAcceleration);
        _0Acceleration = new SlewRateLimiter(constants.Simulation.k0Acceleration);
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        _currentChassisSpeeds =
            fieldRelative ? drive : ChassisSpeeds.fromRobotRelativeSpeeds(drive, RobotState.getInstance().getGyroYaw());

        RobotStateIO.getInstance().setRobotPose(new Pose2d(
            RobotStateIO.getInstance().getRobotPose().getX()
                + _xAcceleration.calculate(_currentChassisSpeeds.vxMetersPerSecond)
                * SwerveConstants.Simulation.kSimToRealSpeedConversion,
            RobotStateIO.getInstance().getRobotPose().getY()
                + _yAcceleration.calculate(_currentChassisSpeeds.vyMetersPerSecond)
                * SwerveConstants.Simulation.kSimToRealSpeedConversion,
            RobotStateIO.getInstance().getRobotPose()
                .getRotation()
                .plus(Rotation2d.fromRadians(_0Acceleration.calculate(_currentChassisSpeeds.omegaRadiansPerSecond)
                    * SwerveConstants.Simulation.kSimToRealSpeedConversion))));
    }

    @Override
    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
        return fieldRelative ? _currentChassisSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(_currentChassisSpeeds, RobotState.getInstance().getGyroYaw());
    }
}
