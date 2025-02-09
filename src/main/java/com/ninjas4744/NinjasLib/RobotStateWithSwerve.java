package com.ninjas4744.NinjasLib;

import com.ctre.phoenix6.hardware.Pigeon2;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import org.littletonrobotics.junction.Logger;

public abstract class RobotStateWithSwerve<StateEnum> extends RobotStateIO<StateEnum>{
    private AHRS navX;
    private Pigeon2 pigeon;
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveKinematics _kinematics;
    private boolean _gyroInverted;
    private FOMCalculator _fomCalculator;
    private int pigeonID = -1;

    private Translation3d pigeonVelocity = new Translation3d();

    public static void setInstance(RobotStateWithSwerve instance, SwerveDriveKinematics kinematics, boolean gyroInverted, FOMCalculator fomCalculator) {
        _instance = instance;
        instance._kinematics = kinematics;
        instance._gyroInverted = gyroInverted;
        instance._fomCalculator = fomCalculator;
        instance.init();
    }

    public static void setInstance(RobotStateWithSwerve instance, SwerveDriveKinematics kinematics, boolean gyroInverted, FOMCalculator fomCalculator, int pigeonID) {
        _instance = instance;
        instance._kinematics = kinematics;
        instance._gyroInverted = gyroInverted;
        instance._fomCalculator = fomCalculator;
        instance.pigeonID = pigeonID;
        instance.init();
    }

    public static RobotStateWithSwerve getInstance() {
        return (RobotStateWithSwerve)RobotStateIO.getInstance();
    }

    @Override
    protected void init(){
        if(!isSimulated()){
            poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getGyroYaw(),
                ((Swerve)SwerveIO.getInstance()).getModulePositions(), new Pose2d());

            if(pigeonID != -1)
                pigeon = new Pigeon2(pigeonID);
            else
                navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
        }else{
            poseEstimator = new SwerveDrivePoseEstimator(_kinematics, new Rotation2d(),
                new SwerveModulePosition[]{
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                    new SwerveModulePosition(0, Rotation2d.fromDegrees(0))
                }, new Pose2d());
        }
    }

    /**
     * @return position of the robot
     */
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getDistanceTo(Pose2d other){
        return other.getTranslation().minus(getRobotPose().getTranslation());
    }

    /**
     * Set where the code thinks the robot is
     *
     * @param pose - the pose to set the robot pose to
     */
    public void setRobotPose(Pose2d pose) {
        Logger.recordOutput("Robot Pose", pose);

        if(!isSimulated())
            poseEstimator.resetPosition(getGyroYaw(), ((Swerve)SwerveIO.getInstance()).getModulePositions(), pose);
        else
            poseEstimator.resetPosition(getGyroYaw(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(0)) }, pose);
    }

    /**
     * Updates the robot pose according to odometry parameters
     *
     * @param modulePositions - The current position of the swerve modules.
     */
    public void updateRobotPose(SwerveModulePosition[] modulePositions) {
        poseEstimator.update(getGyroYaw(), modulePositions);
//        _robotPosePublisher.set(getRobotPose());
        Logger.recordOutput("Robot Pose", getRobotPose());
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

        Logger.recordOutput("Robot Pose", getRobotPose());
    }

    /**
     * @return yaw angle of the robot according to gyro
     */
    public Rotation2d getGyroYaw() {
        if (!isSimulated())
            if(pigeonID != -1)
                return Rotation2d.fromDegrees(_gyroInverted ? -pigeon.getRotation2d().getDegrees() : pigeon.getRotation2d().getDegrees());
            else
                return Rotation2d.fromDegrees(_gyroInverted ? -navX.getAngle() : navX.getAngle());
        else
            return _gyroInverted
                ? getRobotPose().getRotation().unaryMinus()
                : getRobotPose().getRotation();
    }

    public Translation3d getRobotVelocity() {
        if(pigeonID != -1){
            pigeonVelocity = new Translation3d(
                pigeonVelocity.getX() + pigeon.getAccelerationX().getValueAsDouble() * 0.02,
                pigeonVelocity.getY() + pigeon.getAccelerationY().getValueAsDouble() * 0.02,
                pigeonVelocity.getZ() + pigeon.getAccelerationZ().getValueAsDouble() * 0.02
            );
            return pigeonVelocity;
        }
        else
            return new Translation3d(navX.getVelocityX(), navX.getVelocityY(), navX.getVelocityZ());
    }

    /**
     * Resets the gyro angle, sets it to the given angle
     *
     * @param angle - the angle to set the gyro to
     */
    public void resetGyro(Rotation2d angle) {
        if (!isSimulated()) {
            if(pigeonID != -1){
                System.out.print("Gyro: " + pigeon.getRotation2d().getDegrees() + " -> ");
                pigeon.setYaw(angle.getDegrees());
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
}
