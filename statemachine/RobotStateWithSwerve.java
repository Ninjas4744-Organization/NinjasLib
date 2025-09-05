package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.NinjasLib.localization.NinjasSwervePoseTracker;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public abstract class RobotStateWithSwerve<StateEnum> extends RobotStateBase<StateEnum> {
    private final NinjasSwervePoseTracker poseTracker;

    public static RobotStateWithSwerve getInstance() {
        return (RobotStateWithSwerve) RobotStateBase.getInstance();
    }

    /**
     * Create a new RobotStateWithSwerve with navX gyro sensor.
     *
     * @param kinematics The swerve drive kinematics used in the swerve. Used to calculate odometry.
     */
    public RobotStateWithSwerve(SwerveDriveKinematics kinematics) {
        if (Robot.isReal()) {
            poseTracker = new NinjasSwervePoseTracker(kinematics, Swerve.getInstance().getGyro().getYaw(),
                Swerve.getInstance().getModulePositions(), new Pose2d());
        } else {
            poseTracker = new NinjasSwervePoseTracker(kinematics, new Rotation2d(),
                new SwerveModulePosition[]{
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0))
                }, new Pose2d());
        }
    }

    /**
     * @return 2D position of the robot on the field.
     */
    public Pose2d getRobotPose() {
        return poseTracker.getEstimatedPosition();
    }

    /**
     * @param other Another pose to measure distance to.
     * @return Distance between the robot and another pose. Meters.
     */
    public double getDistance(Pose2d other){
        return other.getTranslation().minus(getRobotPose().getTranslation()).getNorm();
    }

    /**
     * @param other Another pose to measure transform to.
     * @return Translation from robot to another pose including dx, dy, da. Field Relative.
     */
    public Transform2d getTransform(Pose2d other){
        return new Transform2d(
            other.getTranslation().minus(getRobotPose().getTranslation()),
            other.getRotation().minus(getRobotPose().getRotation())
        );
    }

    /**
     * @param other Another pose to measure translation to.
     * @return Translation from robot to another pose including dx, dy. Field Relative.
     */
    public Translation2d getTranslation(Pose2d other) {
        return other.getTranslation().minus(getRobotPose().getTranslation());
    }

    /**
     * Set where the code thinks the robot is.
     *
     * @param pose The pose to set the robot pose to.
     */
    public void setRobotPose(Pose2d pose) {
        if (Robot.isReal())
            poseTracker.resetPosition(Swerve.getInstance().getGyro().getYaw(), Swerve.getInstance().getModulePositions(), pose);
        else
            poseTracker.resetPosition(Swerve.getInstance().getGyro().getYaw(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);

        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to odometry parameters.
     *
     * @param modulePositions The current position of the swerve modules.
     */
    public void updateRobotPose(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw) {
        poseTracker.update(gyroYaw, modulePositions);
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to odometry parameters.
     *
     * @param modulePositions The current position of the swerve modules.
     */
    public void updateRobotPoseWithTime(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw, double timestamp) {
        poseTracker.updateWithTime(timestamp, gyroYaw, modulePositions);
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to given vision estimation.
     *
     * @param estimation The vision estimation.
     */
    public void updateRobotPose(VisionOutput estimation, double odometrySTD, double visionSTD) {
        if (!estimation.hasTargets)
            return;

        poseTracker.setMeasurementStdDevs(VecBuilder.fill(odometrySTD, odometrySTD, odometrySTD),
                VecBuilder.fill(visionSTD, visionSTD, visionSTD));

        poseTracker.addVisionMeasurement(
                estimation.robotPose,
                estimation.timestamp
        );

        Logger.recordOutput("Robot Pose", getRobotPose());
    }
}
