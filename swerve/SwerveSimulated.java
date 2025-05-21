package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.dataclasses.SwerveConstants;

public class SwerveSimulated extends SwerveIO {
    private ChassisSpeeds _wantedSpeeds = new ChassisSpeeds();

    public SwerveSimulated(SwerveConstants constants){
        super(constants);
    }

    @Override
    public void driveO(ChassisSpeeds robotRelativeSpeeds) {
        _wantedSpeeds = robotRelativeSpeeds;

        RobotStateWithSwerve.getInstance().setRobotPose(RobotStateWithSwerve.getInstance().getRobotPose().transformBy(
                new Transform2d(
                        robotRelativeSpeeds.vxMetersPerSecond * 0.02,
                        robotRelativeSpeeds.vyMetersPerSecond * 0.02,
                        Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond * 0.02)
                )
        ));
    }

    @Override
    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
        return fieldRelative ? _wantedSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(_wantedSpeeds, RobotStateWithSwerve.getInstance().getGyroYaw());
    }
}
