package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.localization.RobotPose;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;

public class SwerveController {
    private ProfiledPIDController rotationProfiledPID;
    private PIDController rotationPID;
    private PIDController drivePID;
    private SwerveControllerConstants constants;
    private boolean isProfiledRotationPID;

    private SwerveSpeeds lastInput;
    private String channel;
    private String previousChannel;

    private Rotation2d targetAngle = Rotation2d.kZero;
    private double rotationCorrectionLastInput = 0;

    private static SwerveController instance = null;
    private boolean disabled = false;

    public static void setInstance(SwerveController swerveController) {
        instance = swerveController;
    }

    public static SwerveController get() {
        if (instance == null) {
            NinjasLogger.logEventImportant("SwerveController instance not set. Initialize SwerveController by setInstance(SwerveController).");
            return new SwerveController();
        }
        return instance;
    }

    private SwerveController() {
        disabled = true;
    }

    public SwerveController(SwerveControllerConstants constants) {
        if (constants.swerveConstants == null) {
            disabled = true;
            return;
        }

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
            rotationProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
        } else {
            rotationPID = new PIDController(
                constants.rotationPIDConstants.P,
                constants.rotationPIDConstants.I,
                constants.rotationPIDConstants.D
            );
            rotationPID.setIZone(constants.rotationPIDConstants.IZone);
            rotationPID.enableContinuousInput(-Math.PI, Math.PI);
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
        if (disabled)
            return 0;

        if (isProfiledRotationPID)
            return rotationProfiledPID.calculate(RobotPose.get().getRobotPose().getRotation().getRadians(), angle.getRadians());
        return rotationPID.calculate(RobotPose.get().getRobotPose().getRotation().getRadians(), angle.getRadians());
    }

    /**
     * Makes the swerve use PID to look according to the given direction
     *
     * @param direction - the direction vector to look
     */
    public double lookAt(Translation2d direction) {
        if (disabled)
            return 0;

        if (!(direction.getX() == 0 && direction.getY() == 0))
            return lookAt(direction.getAngle());

        return 0;
    }

    public double lookAt(Pose2d target, Rotation2d offset) {
        if (disabled)
            return 0;

        Translation2d lookAtTranslation = RobotPose.get().getTransform(target).getTranslation().rotateBy(offset);
        return lookAt(lookAtTranslation);
    }

    public void resetLookAt() {
        if (disabled)
            return;

        if (isProfiledRotationPID)
            rotationProfiledPID.reset(RobotPose.get().getRobotPose().getRotation().getRadians());
        else
            NinjasLogger.logEvent("Tried to reset a non profiled swerve rotation pid");
    }

    public Translation2d pidTo(Translation2d target) {
        if (disabled)
            return new Translation2d();

        double dist = RobotPose.get().getDistance(new Pose2d(target, Rotation2d.kZero));
        return RobotPose.get().getTranslation(new Pose2d(target, Rotation2d.kZero)).div(dist).times(drivePID.calculate(-dist));
    }

    public void setControl(SwerveSpeeds input, String channel) {
        if (channel.equals(this.channel)) {
            Swerve.get().drive(input);
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
        if (disabled)
            return new SwerveSpeeds();

        return new SwerveSpeeds(
            percent.vxMetersPerSecond * constants.swerveConstants.speeds.maxSpeed,
            percent.vyMetersPerSecond * constants.swerveConstants.speeds.maxSpeed,
            percent.omegaRadiansPerSecond * constants.swerveConstants.speeds.maxAngularVelocity,
            percent.fieldRelative
        );
    }

    public void periodic() {
        if (disabled)
            return;

        Swerve.get().periodic();

        NinjasLogger.log("Swerve/Input", lastInput);
        NinjasLogger.log("Swerve/Channel", channel);
        NinjasLogger.log("Swerve/Previous Channel", previousChannel);
    }
}
