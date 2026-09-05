// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.NinjasLib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * A {@link NinjasPoseTracker} specialized for swerve drivetrains, backed by a {@link
 * SwerveDriveOdometry}. This is the concrete tracker type used by {@link RobotPose} (both for the
 * vision-fused estimate and the odometry-only estimate) and is generally the entry point subsystems
 * should use rather than the generic base class directly.
 */
public class NinjasSwervePoseTracker extends NinjasPoseTracker<SwerveModulePosition[]> {
    private final int m_numModules;

    /**
     * Constructs a swerve pose tracker seeded with an initial gyro angle, module positions, and pose.
     *
     * @param kinematics The swerve drive kinematics for the drivetrain.
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance and rotation measurements of the swerve modules.
     * @param initialPoseMeters The starting pose of the robot on the field.
     */
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

    /**
     * Updates the pose estimate with new gyro and swerve module readings. Should be called every loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param gyroAngle The current gyro angle.
     * @param wheelPositions The current distance and rotation measurements of the swerve modules; must
     *     have the same length as was passed to the constructor.
     * @return The estimated pose of the robot in meters.
     * @throws IllegalArgumentException if {@code wheelPositions.length} does not match the number of
     *     modules the tracker was constructed with.
     */
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
