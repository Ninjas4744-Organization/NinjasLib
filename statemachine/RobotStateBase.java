package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.lib.NinjasLib.localization.NinjasSwervePoseTracker;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public abstract class RobotStateBase {
    private final NinjasSwervePoseTracker poseEstimator;
    private final NinjasSwervePoseTracker odometryOnlyEstimator;
    private static RobotStateBase instance;

    public static RobotStateBase get() {
        if (instance == null)
            throw new RuntimeException("RobotStateBase not initialized. Initialize RobotStateBase by setInstance() first.");
        return instance;
    }

    public static void set(RobotStateBase instance) {
        RobotStateBase.instance = instance;
    }

    /**
     * Create a new RobotStateWithSwerve with navX gyro sensor.
     *
     * @param kinematics The swerve drive kinematics used in the swerve. Used to calculate odometry.
     */
    public RobotStateBase(SwerveDriveKinematics kinematics) {
        if (Robot.isReal()) {
            poseEstimator = new NinjasSwervePoseTracker(kinematics, Swerve.getInstance().getGyro().getYaw(),
                Swerve.getInstance().getModulePositions(), new Pose2d());
            odometryOnlyEstimator = new NinjasSwervePoseTracker(kinematics, Swerve.getInstance().getGyro().getYaw(),
                    Swerve.getInstance().getModulePositions(), new Pose2d());
        } else {
            poseEstimator = new NinjasSwervePoseTracker(kinematics, new Rotation2d(),
                new SwerveModulePosition[]{
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0))
                }, new Pose2d());
            odometryOnlyEstimator = new NinjasSwervePoseTracker(kinematics, new Rotation2d(),
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
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @return 2D position of the robot on the field only according to odometry, vision is not included.
     */
    public Pose2d getOdometryOnlyRobotPose() {
        return odometryOnlyEstimator.getEstimatedPosition();
    }

    /**
     * @param other Another pose to measure distance to.
     * @return Distance between the robot and another pose. Meters.
     */
    public double getDistance(Pose2d other) {
        return other.getTranslation().minus(getRobotPose().getTranslation()).getNorm();
    }

    /**
     * @param other Another pose to measure transform to.
     * @return Translation from robot to another pose including dx, dy, da. Field Relative.
     */
    public Transform2d getTransform(Pose2d other) {
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
     * @return Translation (0, 0), to robot. Field Relative.
     */
    public Translation2d getTranslation() {
        return getRobotPose().getTranslation();
    }

    /**
     * @return Rotation of robot relative to field
     */
    public Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    /**
     * Set where the code thinks the robot is.
     *
     * @param pose The pose to set the robot pose to.
     */
    public void setRobotPose(Pose2d pose) {
        if (Robot.isReal()){
            poseEstimator.resetPosition(Swerve.getInstance().getGyro().getYaw(), Swerve.getInstance().getModulePositions(), pose);
            odometryOnlyEstimator.resetPosition(Swerve.getInstance().getGyro().getYaw(), Swerve.getInstance().getModulePositions(), pose);
        } else {
            poseEstimator.resetPosition(Swerve.getInstance().getGyro().getYaw(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);

            odometryOnlyEstimator.resetPosition(Swerve.getInstance().getGyro().getYaw(), new SwerveModulePosition[]{
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);
        }

        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    public void setOdometryOnlyRobotPose(Pose2d pose) {
        if (Robot.isReal()){
            odometryOnlyEstimator.resetPosition(Swerve.getInstance().getGyro().getYaw(), Swerve.getInstance().getModulePositions(), pose);
        } else {
            odometryOnlyEstimator.resetPosition(Swerve.getInstance().getGyro().getYaw(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);
        }

        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    public void resetGyro(Rotation2d yaw) {
        Pose2d currentPose = getRobotPose();
        Swerve.getInstance().getGyro().resetYaw(yaw);
        poseEstimator.resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), yaw));
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to odometry parameters.
     *
     * @param modulePositions The current position of the swerve modules.
     */
    public void updateRobotPose(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw) {
        poseEstimator.update(gyroYaw, modulePositions);
        odometryOnlyEstimator.update(gyroYaw, modulePositions);
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to odometry parameters.
     *
     * @param modulePositions The current position of the swerve modules.
     */
    public void updateRobotPoseWithTime(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw, double timestamp) {
        poseEstimator.updateWithTime(timestamp, gyroYaw, modulePositions);
        odometryOnlyEstimator.updateWithTime(timestamp, gyroYaw, modulePositions);
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to given vision estimation.
     *
     * @param estimation The vision estimation.
     */
    public void updateRobotPose(Pose2d estimation, double timestamp, Matrix<N3, N1> visionStrength) {
        poseEstimator.addVisionMeasurement(estimation, timestamp, visionStrength);
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * @return Whether the robot is in the blue alliance or the red alliance
     */
    public static Optional<DriverStation.Alliance> getAlliance() {
        return Robot.isSimulation()
            ? (DriverStationSim.getAllianceStationId().ordinal() > 3
            ? Optional.of(DriverStation.Alliance.Blue)
            : Optional.of(DriverStation.Alliance.Red))
            : DriverStation.getAlliance();
    }

    /**
     * @return Which driver station the robot is in
     */
    public static AllianceStationID getAllianceStation() {
        return Robot.isSimulation() ? DriverStationSim.getAllianceStationId() : DriverStation.getRawAllianceStation();
    }
}
