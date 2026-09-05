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

/**
 * The recommended front door for driving the swerve: layers rotation-to-angle PID
 * ({@link #lookAt}), point-to-point translation PID ({@link #pidTo}), and a named-channel gate
 * ({@link #setControl}/{@link #setChannel}) on top of the raw {@link Swerve} drivetrain, so only
 * whichever command currently "owns" the requested channel can actually move the robot.
 */
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

    /** Sets the singleton instance returned by {@link #get()}. Call once during robot init. */
    public static void setInstance(SwerveController swerveController) {
        instance = swerveController;
    }

    /**
     * @return the singleton {@link SwerveController} instance set via {@link #setInstance}, or a
     *         disabled no-op instance (logging a warning) if none has been set yet
     */
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

    /**
     * Builds the rotation PID (profiled if {@code cruiseVelocity}/{@code acceleration} are set in
     * {@link SwerveControllerConstants#rotationPIDConstants}, otherwise a plain continuous-input
     * PID) and the drive PID from {@code constants}. Disabled if
     * {@link SwerveControllerConstants#swerveConstants} is {@code null}.
     *
     * @param constants the rotation/drive PID gains and underlying swerve configuration
     */
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
     * PID-calculates the rotational velocity (rad/s) needed to turn the robot to face {@code angle}.
     * Does not drive the swerve; feed the result into {@link Swerve#drive}.
     *
     * @param angle the field-relative angle to face
     * @return rotational velocity in radians/second, or {@code 0} if disabled
     */
    public double lookAt(Rotation2d angle) {
        if (disabled)
            return 0;

        if (isProfiledRotationPID)
            return rotationProfiledPID.calculate(RobotPose.get().getRobotPose().getRotation().getRadians(), angle.getRadians());
        return rotationPID.calculate(RobotPose.get().getRobotPose().getRotation().getRadians(), angle.getRadians());
    }

    /**
     * Same as {@link #lookAt(Rotation2d)}, but takes a direction vector instead of an angle.
     * The zero vector has no defined angle, so it's treated as "no target" and returns {@code 0}.
     *
     * @param direction the field-relative direction to face
     * @return rotational velocity in radians/second, or {@code 0} if disabled or {@code direction} is zero
     */
    public double lookAt(Translation2d direction) {
        if (disabled)
            return 0;

        if (!(direction.getX() == 0 && direction.getY() == 0))
            return lookAt(direction.getAngle());

        return 0;
    }

    /**
     * Same as {@link #lookAt(Rotation2d)}, but faces a field-relative {@code target} pose (e.g. a
     * goal), skewed by {@code offset} to aim a mechanism that isn't robot-front-facing.
     *
     * @param target the field-relative pose to look at
     * @param offset extra rotation applied to the look-at direction before aiming
     * @return rotational velocity in radians/second, or {@code 0} if disabled
     */
    public double lookAt(Pose2d target, Rotation2d offset) {
        if (disabled)
            return 0;

        Translation2d lookAtTranslation = RobotPose.get().getTransform(target).getTranslation().rotateBy(offset);
        return lookAt(lookAtTranslation);
    }

    /**
     * Resets the profiled rotation PID's state to the robot's current heading, so the next
     * {@link #lookAt} call doesn't use a stale setpoint/velocity from a previous target. Only
     * meaningful when the profiled rotation PID is in use; otherwise logs and does nothing, since
     * a plain {@link PIDController} has no motion-profile state to reset.
     */
    public void resetLookAt() {
        if (disabled)
            return;

        if (isProfiledRotationPID)
            rotationProfiledPID.reset(RobotPose.get().getRobotPose().getRotation().getRadians());
        else
            NinjasLogger.logEvent("Tried to reset a non profiled swerve rotation pid");
    }

    /**
     * PID-calculates a velocity vector, in m/s, that drives the robot straight towards
     * {@code target}. Does not drive the swerve; feed the result into {@link Swerve#drive}.
     *
     * @param target the field-relative point to drive towards
     * @return a velocity vector pointing at {@code target}, or the zero vector if disabled
     */
    public Translation2d pidTo(Translation2d target) {
        if (disabled)
            return new Translation2d();

        double dist = RobotPose.get().getDistance(new Pose2d(target, Rotation2d.kZero));
        return RobotPose.get().getTranslation(new Pose2d(target, Rotation2d.kZero)).div(dist).times(drivePID.calculate(-dist));
    }

    /**
     * Drives the swerve with {@code input}, but only if {@code channel} matches the currently
     * active channel (see {@link #setChannel}). This is the gate that lets multiple commands share
     * the drivetrain safely: a command that isn't the current channel owner silently has no effect.
     *
     * @param input   the speeds to drive with if this call is authorized
     * @param channel the channel the caller believes it owns
     */
    public void setControl(SwerveSpeeds input, String channel) {
        if (channel.equals(this.channel)) {
            Swerve.get().drive(input);
            lastInput = input;
        }
    }

    /**
     * Switches which channel is allowed to drive the swerve via {@link #setControl}, remembering
     * the old one in {@link #getPreviousChannel()}.
     *
     * @param channel the channel to make active
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

    /** @return the speeds last passed to {@link #setControl} by the current channel owner */
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

    /**
     * Must be called once per robot loop cycle. Delegates to {@link Swerve#periodic()} and logs
     * the current input/channel state. A no-op if this controller is disabled.
     */
    public void periodic() {
        if (disabled)
            return;

        Swerve.get().periodic();

        NinjasLogger.log("Swerve/Input", lastInput);
        NinjasLogger.log("Swerve/Channel", channel);
        NinjasLogger.log("Swerve/Previous Channel", previousChannel);
    }
}
