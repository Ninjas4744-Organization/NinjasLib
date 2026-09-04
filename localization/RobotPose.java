package frc.lib.NinjasLib.localization;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Robot;

import java.util.Optional;

public class RobotPose {
    private NinjasSwervePoseTracker poseEstimator;
    private NinjasSwervePoseTracker odometryOnlyEstimator;

    private VisionStrengthCalculator visionStrengthCalculator;
    private VisionFiltersCalculator visionFiltersCalculator;

    private static RobotPose instance;
    private boolean disabled = false;

    public static RobotPose get() {
        if (instance == null) {
            NinjasLogger.logEventImportant("RobotPose instance not set. Set robot pose instance by setInstance(RobotPose instance).");
            return new RobotPose(); // Disabled RobotPose
        }
        return instance;
    }

    public static void setInstance(RobotPose instance) {
        RobotPose.instance = instance;
    }

    private RobotPose() {
        disabled = true;
    }

    /**
     * Create a new RobotStateWithSwerve with navX gyro sensor.
     *
     * @param kinematics The swerve drive kinematics used in the swerve. Used to calculate odometry.
     */
    public RobotPose(SwerveDriveKinematics kinematics, VisionStrengthCalculator visionStrengthCalculator, VisionFiltersCalculator visionFiltersCalculator) {
        this.visionStrengthCalculator = visionStrengthCalculator;
        this.visionFiltersCalculator = visionFiltersCalculator;

        if (Robot.isReal()) {
            poseEstimator = new NinjasSwervePoseTracker(kinematics, Swerve.get().getGyro().getYaw(),
                Swerve.get().getModulePositions(), new Pose2d());
            odometryOnlyEstimator = new NinjasSwervePoseTracker(kinematics, Swerve.get().getGyro().getYaw(),
                    Swerve.get().getModulePositions(), new Pose2d());
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
        if (disabled)
            return new Pose2d();

        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @return 2D position of the robot on the field only according to odometry, vision is not included.
     */
    public Pose2d getOdometryOnlyRobotPose() {
        if (disabled)
            return new Pose2d();

        return odometryOnlyEstimator.getEstimatedPosition();
    }

    /**
     * @param other Another pose to measure distance to.
     * @return Distance between the robot and another pose. Meters.
     */
    public double getDistance(Pose2d other) {
        if (disabled)
            return 0;

        return other.getTranslation().minus(getRobotPose().getTranslation()).getNorm();
    }

    /**
     * @param other Another pose to measure transform to.
     * @return Translation from robot to another pose including dx, dy, da. Field Relative.
     */
    public Transform2d getTransform(Pose2d other) {
        if (disabled)
            return new Transform2d();

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
        if (disabled)
            return new Translation2d();

        return other.getTranslation().minus(getRobotPose().getTranslation());
    }

    /**
     * @return Translation (0, 0), to robot. Field Relative.
     */
    public Translation2d getTranslation() {
        if (disabled)
            return new Translation2d();

        return getRobotPose().getTranslation();
    }

    /**
     * @return Rotation of robot relative to field
     */
    public Rotation2d getRotation() {
        if (disabled)
            return new Rotation2d();

        return getRobotPose().getRotation();
    }

    /**
     * Set where the code thinks the robot is.
     *
     * @param pose The pose to set the robot pose to.
     */
    public void setRobotPose(Pose2d pose) {
        if (disabled)
            return;

        if (Robot.isReal()){
            poseEstimator.resetPose(pose);
            odometryOnlyEstimator.resetPose(pose);
        } else {
            poseEstimator.resetPosition(Swerve.get().getGyro().getYaw(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);

            odometryOnlyEstimator.resetPosition(Swerve.get().getGyro().getYaw(), new SwerveModulePosition[]{
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);
        }

        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    public void setOdometryOnlyRobotPose(Pose2d pose) {
        if (disabled)
            return;

        if (Robot.isReal()){
            odometryOnlyEstimator.resetPosition(Swerve.get().getGyro().getYawOffsetted(), Swerve.get().getModulePositions(), pose);
        } else {
            odometryOnlyEstimator.resetPosition(Swerve.get().getGyro().getYawOffsetted(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0))}, pose);
        }

        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    public void resetGyro(Rotation2d yaw) {
        if (disabled)
            return;

        Pose2d currentPose = getRobotPose();
        Swerve.get().getGyro().resetYaw(yaw);
        poseEstimator.resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), yaw));
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to odometry parameters.
     *
     * @param modulePositions The current position of the swerve modules.
     */
    public void addOdometryUpdate(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw) {
        if (disabled)
            return;

        poseEstimator.update(gyroYaw, modulePositions);
        odometryOnlyEstimator.update(gyroYaw, modulePositions);
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to odometry parameters.
     *
     * @param modulePositions The current position of the swerve modules.
     */
    public void addTimedOdometryUpdate(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw, double timestamp) {
        if (disabled)
            return;

        poseEstimator.updateWithTime(timestamp, gyroYaw, modulePositions);
        odometryOnlyEstimator.updateWithTime(timestamp, gyroYaw, modulePositions);
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to given vision estimation.
     *
     * @param estimation The vision estimation.
     */
    public void addManualVisionUpdate(Pose2d estimation, double timestamp, Matrix<N3, N1> visionStrength) {
        if (disabled)
            return;

        poseEstimator.addVisionMeasurement(estimation, timestamp, visionStrength);
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    public void addVisionUpdate(VisionOutput estimation, double timestamp) {
        if (!estimation.hasTargets || disabled)
            return;

        boolean passedFilters = visionFiltersCalculator.isPassed(estimation);
        Matrix<N3, N1> strength = visionStrengthCalculator.calculate(estimation);

        if (passedFilters) {
            addManualVisionUpdate(estimation.robotPose, timestamp, strength);
        }

        NinjasLogger.log("Vision/" + estimation.cameraName + "/Passed Filters", passedFilters);
        NinjasLogger.log("Vision/" + estimation.cameraName + "/Strength", strength);
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
