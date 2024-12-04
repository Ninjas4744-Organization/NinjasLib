package com.ninjas4744.NinjasLib;

import com.ninjas4744.NinjasLib.DataClasses.FOMCalculator;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.studica.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;

public abstract class RobotStateWithSwerve<StateEnum> extends RobotStateIO<StateEnum>{
    private AHRS navX;
    private SwerveDrivePoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> _robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Robot Pose", Pose2d.struct)
      .publish();
    private SwerveDriveKinematics _kinematics;
    private boolean _gyroInverted;
    private FOMCalculator _fomCalculator;

    public static void setInstance(RobotStateWithSwerve instance, TimedRobot robot, SwerveDriveKinematics kinematics, boolean gyroInverted, FOMCalculator fomCalculator){
        _instance = instance;
        instance._robot = robot;
        instance._kinematics = kinematics;
        instance._gyroInverted = gyroInverted;
        instance._fomCalculator = fomCalculator;
        instance.init();
    }

    public static RobotStateWithSwerve getInstance() {
        return (RobotStateWithSwerve)RobotStateIO.getInstance();
    }

    @Override
    protected void init(){
        navX = new AHRS(AHRS.NavXComType.kMXP_UART);

        if(!isSimulated())
            poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getGyroYaw(),
              ((Swerve)SwerveIO.getInstance()).getModulePositions(), new Pose2d());
    }

    /**
     * @return position of the robot
     */
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Set where the code thinks the robot is
     *
     * @param pose - the pose to set the robot pose to
     */
    public void setRobotPose(Pose2d pose) {
        _robotPosePublisher.set(pose);

        if(!isSimulated())
            poseEstimator.resetPosition(getGyroYaw(), ((Swerve)SwerveIO.getInstance()).getModulePositions(), pose);
    }

    /**
     * Updates the robot pose according to odometry parameters
     *
     * @param modulePositions - The current position of the swerve modules.
     */
    public void updateRobotPose(SwerveModulePosition[] modulePositions) {
        poseEstimator.update(getGyroYaw(), modulePositions);
        _robotPosePublisher.set(getRobotPose());
    }

    /**
     * Updates the robot pose according to the given vision estimation
     *
     * @param visionEstimation - the estimation
     */
    public void updateRobotPose(VisionOutput visionEstimation) {
        if (visionEstimation.hasTargets){
            poseEstimator.addVisionMeasurement(
              visionEstimation.robotPose,
              visionEstimation.timestamp,
              new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
                _fomCalculator.calculateFOM(visionEstimation),
                _fomCalculator.calculateFOM(visionEstimation),
                _fomCalculator.calculateFOM(visionEstimation),
              })
            );
        }

        _robotPosePublisher.set(getRobotPose());
    }

    /**
     * @return yaw angle of the robot according to gyro
     */
    public Rotation2d getGyroYaw() {
        if (!isSimulated())
            return Rotation2d.fromDegrees(_gyroInverted ? -navX.getAngle() : navX.getAngle());
        else
            return _gyroInverted
              ? getRobotPose().getRotation().unaryMinus()
              : getRobotPose().getRotation();
    }

    public Translation3d getRobotVelocity() {
        return new Translation3d(navX.getVelocityX(), navX.getVelocityY(), navX.getVelocityZ());
    }

    /**
     * Resets the gyro angle, sets it to the given angle
     *
     * @param angle - the angle to set the gyro to
     */
    public void resetGyro(Rotation2d angle) {
        if (!isSimulated()) {
            System.out.print("Gyro: " + navX.getAngle() + " -> ");
            navX.reset();
            navX.setAngleAdjustment(angle.getDegrees());
            System.out.println(navX.getAngle());
        } else {
            System.out.print("Gyro: " + getRobotPose().getRotation().getDegrees() + " -> ");
            setRobotPose(new Pose2d(getRobotPose().getTranslation(), angle));
            System.out.println(getRobotPose().getRotation().getDegrees());
        }
    }
}
