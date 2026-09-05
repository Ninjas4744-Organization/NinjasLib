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

/**
 * The robot's single source of truth for "where am I on the field". This is a singleton facade
 * (accessed via {@link #get()}) that every subsystem and command should query instead of maintaining
 * its own pose estimate. Internally it keeps two {@link NinjasSwervePoseTracker}s in lockstep: a
 * vision-fused estimate ({@link #getRobotPose()}) and an odometry-only estimate ({@link
 * #getOdometryOnlyRobotPose()}) that ignores vision entirely, and delegates per-camera trust and
 * sanity checks to the injected {@link VisionStrengthCalculator} and {@link VisionFiltersCalculator}.
 *
 * <p>If no instance has been set with {@link #setInstance}, {@link #get()} returns a disabled
 * placeholder whose queries return default/zero values and whose updates are no-ops, so callers never
 * need to null-check it.
 */
public class RobotPose {
    private NinjasSwervePoseTracker poseEstimator;
    private NinjasSwervePoseTracker odometryOnlyEstimator;

    private VisionStrengthCalculator visionStrengthCalculator;
    private VisionFiltersCalculator visionFiltersCalculator;

    private static RobotPose instance;
    private boolean disabled = false;

    /**
     * Returns the shared {@code RobotPose} instance that the rest of the robot code should query. If
     * {@link #setInstance} has not been called yet, logs an important event and returns a disabled
     * instance (all queries return default/zero values, all updates are no-ops) so callers can use it
     * safely without a null check.
     *
     * @return The active {@code RobotPose} instance, or a disabled placeholder if none was set.
     */
    public static RobotPose get() {
        if (instance == null) {
            NinjasLogger.logEventImportant("RobotPose instance not set. Set robot pose instance by setInstance(RobotPose instance).");
            return new RobotPose(); // Disabled RobotPose
        }
        return instance;
    }

    /**
     * Installs the {@code RobotPose} instance returned by future calls to {@link #get()}. Should be
     * called once during robot initialization, before any subsystem queries the robot's pose.
     *
     * @param instance The instance to install as the singleton.
     */
    public static void setInstance(RobotPose instance) {
        RobotPose.instance = instance;
    }

    private RobotPose() {
        disabled = true;
    }

    /**
     * Constructs an enabled {@code RobotPose}, creating its vision-fused and odometry-only swerve pose
     * trackers. On a real robot the trackers are seeded from the current gyro yaw and module positions
     * via {@link Swerve#get()}; in simulation they are seeded with a zeroed gyro and module set, since
     * {@link Swerve#get()} isn't available yet at construction time. Pass the result to {@link
     * #setInstance} to make it the active instance.
     *
     * @param kinematics The swerve drive kinematics used to calculate odometry.
     * @param visionStrengthCalculator Determines how much to trust each incoming vision measurement.
     * @param visionFiltersCalculator Determines whether an incoming vision measurement should be
     *     rejected outright before it reaches the pose trackers.
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
     * The main "where is the robot" query — the vision-fused pose estimate that the rest of the robot
     * code should use by default (for driving to targets, logging, autonomous, etc.).
     *
     * @return 2D position of the robot on the field, or the identity pose if disabled.
     */
    public Pose2d getRobotPose() {
        if (disabled)
            return new Pose2d();

        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Use when a discontinuity from a vision correction would be undesirable (e.g. as a smooth input
     * to a velocity/acceleration-based control loop), at the cost of accumulating odometry drift over
     * time.
     *
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
     * Resets both the vision-fused and odometry-only pose estimates to the given pose (e.g. when
     * placing the robot at a known starting position, or applying a full pose correction). On a real
     * robot this keeps the current gyro yaw; in simulation, module positions are reset to zero.
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

    /**
     * Resets only the odometry-only pose estimate to the given pose, leaving the vision-fused estimate
     * untouched.
     *
     * @param pose The pose to set the odometry-only estimate to.
     */
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

    /**
     * Re-zeros the physical gyro to the given yaw and immediately re-anchors the vision-fused pose
     * estimate's rotation to match, so the reported heading doesn't jump on the next odometry update.
     * Does not touch the odometry-only estimate.
     *
     * @param yaw The yaw to reset the gyro to.
     */
    public void resetGyro(Rotation2d yaw) {
        if (disabled)
            return;

        Pose2d currentPose = getRobotPose();
        Swerve.get().getGyro().resetYaw(yaw);
        poseEstimator.resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), yaw));
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * Feeds a new wheel/gyro sample into both pose trackers, using the current time as the sample's
     * timestamp. This is the main per-loop entry point for keeping the robot's pose current — call it
     * once per periodic cycle with the latest module and gyro readings. Use {@link
     * #addTimedOdometryUpdate} instead when replaying higher-frequency samples (e.g. from {@link
     * OdometryThread}) that carry their own timestamps.
     *
     * @param modulePositions The current position of the swerve modules.
     * @param gyroYaw The current gyro yaw.
     */
    public void addOdometryUpdate(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw) {
        if (disabled)
            return;

        poseEstimator.update(gyroYaw, modulePositions);
        odometryOnlyEstimator.update(gyroYaw, modulePositions);
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * Same as {@link #addOdometryUpdate}, but with an explicit sample timestamp. Use this when
     * replaying a backlog of samples captured at a higher frequency than the main loop (e.g. from
     * {@link OdometryThread}'s queues), so each sample is correctly ordered relative to buffered vision
     * corrections instead of all being stamped with the current time.
     *
     * @param modulePositions The current position of the swerve modules.
     * @param gyroYaw The gyro yaw at the time of this sample.
     * @param timestamp The timestamp of this sample, in seconds (FPGA time).
     */
    public void addTimedOdometryUpdate(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw, double timestamp) {
        if (disabled)
            return;

        poseEstimator.updateWithTime(timestamp, gyroYaw, modulePositions);
        odometryOnlyEstimator.updateWithTime(timestamp, gyroYaw, modulePositions);
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * Applies a vision measurement directly to the vision-fused pose tracker, bypassing the configured
     * {@link VisionFiltersCalculator} and {@link VisionStrengthCalculator}. Prefer {@link
     * #addVisionUpdate(VisionOutput, double)} for normal camera pipelines; use this when the caller has
     * already decided the measurement is valid and how much to trust it.
     *
     * @param estimation The vision-measured robot pose.
     * @param timestamp The timestamp of the measurement, in seconds (FPGA time).
     * @param visionStrength Per-axis trust to apply to this measurement (x, y, theta); see {@link
     *     NinjasPoseTracker#setVisionMeasurementStrength}.
     */
    public void addManualVisionUpdate(Pose2d estimation, double timestamp, Matrix<N3, N1> visionStrength) {
        if (disabled)
            return;

        poseEstimator.addVisionMeasurement(estimation, timestamp, visionStrength);
        NinjasLogger.log("Robot Pose", getRobotPose());
    }

    /**
     * The standard entry point for feeding a camera pipeline's output into the robot's pose estimate.
     * This is what vision subsystems should call once per camera result; it is the piece that decides
     * whether a vision measurement is trustworthy and, if so, how much.
     * <p>
     * Does nothing if the estimation reports no targets. Otherwise, runs the configured {@link
     * VisionFiltersCalculator} to sanity-check the measurement (e.g. reject implausible poses) and the
     * configured {@link VisionStrengthCalculator} to compute per-axis trust (e.g. weighted by distance
     * to target and current odometry drift); only measurements that pass the filter are applied, via
     * {@link #addManualVisionUpdate}. Whether the measurement passed and its computed strength are
     * always logged, even when rejected.
     *
     * @param estimation One camera's vision pose estimate for this cycle.
     * @param timestamp The timestamp of the measurement, in seconds (FPGA time).
     */
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
