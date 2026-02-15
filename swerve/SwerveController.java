package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveController {
    private ProfiledPIDController rotationProfiledPID;
    private PIDController rotationPID;
    private final PIDController drivePID;
    private final SwerveControllerConstants constants;
    private final boolean isProfiledRotationPID;

    private SwerveSpeeds lastInput;
    private String channel;
    private String previousChannel;

    private Rotation2d targetAngle = Rotation2d.kZero;
    private double rotationCorrectionLastInput = 0;

    private static SwerveController instance = null;

    public static void setInstance(SwerveController swerveController) {
        instance = swerveController;
    }

    public static SwerveController getInstance() {
        if (instance == null)
            throw new RuntimeException("SwerveController constants not given. Initialize SwerveController by setConstants(SwerveControllerConstants, SwerveIO) first.");
        return instance;
    }

    public SwerveController(SwerveControllerConstants constants) {
        this.constants = constants;

        channel = "";
        previousChannel = "";
        lastInput = new SwerveSpeeds();

        isProfiledRotationPID = !(constants.rotationPIDConstants.cruiseVelocity == 0
                && constants.rotationPIDConstants.acceleration == 0);

        if (isProfiledRotationPID) {
            rotationProfiledPID = new ProfiledPIDController(
                constants.rotationPIDConstants.P,
                constants.rotationPIDConstants.I,
                constants.rotationPIDConstants.D,
                new TrapezoidProfile.Constraints(constants.rotationPIDConstants.cruiseVelocity, constants.rotationPIDConstants.acceleration)
            );
            rotationProfiledPID.setIZone(constants.rotationPIDConstants.IZone);
            rotationProfiledPID.enableContinuousInput(constants.rotationPIDContinuousConnections.getFirst(), constants.rotationPIDContinuousConnections.getSecond());
        } else {
            rotationPID = new PIDController(
                constants.rotationPIDConstants.P,
                constants.rotationPIDConstants.I,
                constants.rotationPIDConstants.D
            );
            rotationPID.setIZone(constants.rotationPIDConstants.IZone);
            rotationPID.enableContinuousInput(constants.rotationPIDContinuousConnections.getFirst(), constants.rotationPIDContinuousConnections.getSecond());
        }

        drivePID = new PIDController(
                constants.drivePIDConstants.P,
                constants.drivePIDConstants.I,
                constants.drivePIDConstants.D);
        drivePID.setIZone(constants.drivePIDConstants.IZone);
    }

    /**
     * Makes the swerve use PID to look at the given angle
     *
     * @param angle the angle to look at
     */
    public double lookAt(Rotation2d angle) {
        if (isProfiledRotationPID)
            return rotationProfiledPID.calculate(RobotStateWithSwerve.getInstance().getRobotPose().getRotation().getRadians(), angle.getRadians());
        return rotationPID.calculate(RobotStateWithSwerve.getInstance().getRobotPose().getRotation().getRadians(), angle.getRadians());
    }

    /**
     * Makes the swerve use PID to look according to the given direction
     *
     * @param direction - the direction vector to look
     */
    public double lookAt(Translation2d direction) {
        if (!(direction.getX() == 0 && direction.getY() == 0))
            return lookAt(direction.getAngle());

        return 0;
    }

    public double lookAt(Pose2d target, Rotation2d offset) {
        Translation2d lookAtTranslation = RobotStateWithSwerve.getInstance().getTransform(target).getTranslation().rotateBy(offset);
        return lookAt(lookAtTranslation);
    }

    public void resetLookAt() {
        if (isProfiledRotationPID)
            rotationProfiledPID.reset(RobotStateWithSwerve.getInstance().getRobotPose().getRotation().getRadians());
        else
            System.out.println("Tried to reset a non profiled swerve rotation pid");
    }

    public Translation2d pidTo(Translation2d target) {
        double dist = RobotStateWithSwerve.getInstance().getDistance(new Pose2d(target, Rotation2d.kZero));
        return RobotStateWithSwerve.getInstance().getTranslation(new Pose2d(target, Rotation2d.kZero)).div(dist).times(drivePID.calculate(-dist));
    }

    public void setControl(SwerveSpeeds input, String channel) {
        if (channel.equals(this.channel)) {
            Swerve.getInstance().drive(input);
            lastInput = input;
        }
    }

    /**
     * Set the current state of the swerve, so it will work according
     * @param channel the wanted state
     */
    public void setChannel(String channel) {
        previousChannel = this.channel;
        this.channel = channel;
    }

    /**
     * @return the current state of the swerve
     */
    public String getChannel() {
        return channel;
    }

    /**
     * @return the previous state of the swerve, the state it was before changing it
     */
    public String getPreviousChannel() {
        return previousChannel;
    }

    public SwerveSpeeds getLastInput() {
        return lastInput;
    }

    /**
     * Convert percent chassis speeds to m/s chassis speeds
     * @param percent the percent chassis speeds to convert
     * @return the m/s chassis speeds to give the swerve
     */
    public SwerveSpeeds fromPercent(SwerveSpeeds percent) {
        return new SwerveSpeeds(
            percent.vxMetersPerSecond * constants.swerveConstants.limits.maxSpeed,
            percent.vyMetersPerSecond * constants.swerveConstants.limits.maxSpeed,
            percent.omegaRadiansPerSecond * constants.swerveConstants.limits.maxAngularVelocity,
            percent.fieldRelative
        );
    }

    public void periodic() {
        Swerve.getInstance().periodic();

        Logger.recordOutput("Swerve/Input", lastInput);
        Logger.recordOutput("Swerve/Channel", channel);
        Logger.recordOutput("Swerve/Previous Channel", previousChannel);
    }
}
