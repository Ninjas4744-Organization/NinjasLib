package frc.lib.NinjasLib;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.NinjasLib.dataclasses.VisionOutput;
import frc.lib.NinjasLib.swerve.NinjasSwervePoseTracker;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

public abstract class RobotStateWithSwerve<StateEnum> extends RobotStateBase<StateEnum> {
    private AHRS navX;
    private Pigeon2 pigeon;
    //    private final SwerveDrivePoseEstimator poseEstimator;
    private final NinjasSwervePoseTracker poseTracker;
    private final boolean gyroInverted;
    private int pigeonID = -1;

    public static RobotStateWithSwerve getInstance() {
        return (RobotStateWithSwerve) RobotStateBase.getInstance();
    }

    /**
     * Create a new RobotStateWithSwerve with navX gyro sensor.
     *
     * @param kinematics    The swerve drive kinematics used in the swerve. Used to calculate odometry.
     * @param gyroInverted  Whether to invert the returned angle of the gyro. Counterclockwise positive if not inverted.
     */
    public RobotStateWithSwerve(SwerveDriveKinematics kinematics, boolean gyroInverted) {
        this.gyroInverted = gyroInverted;

        if (Robot.isReal()) {
            navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

            poseTracker = new NinjasSwervePoseTracker(kinematics, getGyroYaw(),
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
     * Create a new RobotStateWithSwerve with pigeon gyro sensor.
     *
     * @param kinematics    The swerve drive kinematics used in the swerve. Used to calculate odometry.
     * @param gyroInverted  Whether to invert the returned angle of the gyro. Counterclockwise positive if not inverted.
     * @param pigeonID      The pigeon gyro sensor CAN id.
     */
    public RobotStateWithSwerve(SwerveDriveKinematics kinematics, boolean gyroInverted, int pigeonID) {
        this.gyroInverted = gyroInverted;
        this.pigeonID = pigeonID;

        if (Robot.isReal()) {
            pigeon = new Pigeon2(pigeonID);

            poseTracker = new NinjasSwervePoseTracker(kinematics, getGyroYaw(),
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
            poseTracker.resetPosition(getGyroYaw(), Swerve.getInstance().getModulePositions(), pose);
        else
            poseTracker.resetPosition(getGyroYaw(), new SwerveModulePosition[]{
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
    public void updateRobotPose(SwerveModulePosition[] modulePositions) {
        poseTracker.update(getGyroYaw(), modulePositions);
        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * Updates the robot pose according to given vision estimation.
     *
     * @param estimation The vision estimation.
     */
    public void updateRobotPose(VisionOutput estimation, double odometryFOM, double visionFOM) {
        if (!estimation.hasTargets)
            return;

        poseTracker.setMeasurementStdDevs(VecBuilder.fill(odometryFOM, odometryFOM, odometryFOM),
                VecBuilder.fill(visionFOM, visionFOM, visionFOM));

        poseTracker.addVisionMeasurement(
                estimation.robotPose,
                estimation.timestamp
        );

        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * @return Yaw angle of the robot according to gyro.
     */
    public Rotation2d getGyroYaw() {
        if (Robot.isReal())
            if(pigeonID != -1)
                return Rotation2d.fromDegrees(gyroInverted ? -pigeon.getRotation2d().getDegrees() : pigeon.getRotation2d().getDegrees());
            else
                return Rotation2d.fromDegrees(gyroInverted ? -navX.getAngle() : navX.getAngle());
        else
            return gyroInverted
                ? Swerve.getInstance().getGyroSimulationReading().unaryMinus()
                : Swerve.getInstance().getGyroSimulationReading();
    }

    /**
     * Resets the gyro angle, sets it to the given angle.
     *
     * @param angle The angle to set the gyro to.
     */
    public void resetGyro(Rotation2d angle) {
        if (Robot.isReal()) {
            if(pigeonID != -1){
                System.out.print("Gyro: " + pigeon.getRotation2d().getDegrees() + " -> ");
                pigeon.setYaw(angle.getDegrees(), 0);
                System.out.println(pigeon.getRotation2d().getDegrees());
            }
            else{
                System.out.print("Gyro: " + navX.getAngle() + " -> ");
                navX.reset();
                navX.setAngleAdjustment(angle.getDegrees());
                System.out.println(navX.getAngle());
            }
        } else {
            System.out.print("Gyro: " + getRobotPose().getRotation().getDegrees() + " -> ");
            setRobotPose(new Pose2d(getRobotPose().getTranslation(), angle));
            System.out.println(getRobotPose().getRotation().getDegrees());
        }
    }

    public Translation2d getAcceleration() {
        if (pigeonID != -1)
            return new Translation2d(pigeon.getAccelerationX().getValue().in(MetersPerSecondPerSecond), pigeon.getAccelerationY().getValue().in(MetersPerSecondPerSecond));
        return new Translation2d(navX.getWorldLinearAccelX() * 9.806, navX.getWorldLinearAccelY() * 9.806);
    }
}
