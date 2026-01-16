// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.NinjasLib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NinjasSwervePoseTracker extends NinjasPoseTracker<SwerveModulePosition[]> {
    private final int m_numModules;

    public NinjasSwervePoseTracker(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters) {
        super(
            kinematics,
            new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters));

        m_numModules = modulePositions.length;
    }

    @Override
    public Pose2d updateWithTime(
        double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        if (wheelPositions.length != m_numModules) {
            throw new IllegalArgumentException(
                "Number of modules is not consistent with number of wheel locations provided in "
                    + "constructor");
        }

        return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    }
}
