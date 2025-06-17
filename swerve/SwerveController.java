package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.dataclasses.SwerveControllerConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveController {
    private final PIDController anglePID;
    private final PIDController xPID;
    private final PIDController yPID;
    private final SwerveControllerConstants constants;

    private ChassisSpeeds lastInput;
    private String channel;
    private String previousChannel;

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
        lastInput = new ChassisSpeeds();

        anglePID = new PIDController(
            constants.rotationPIDConstants.P,
            constants.rotationPIDConstants.I,
            constants.rotationPIDConstants.D
        );
        anglePID.setIZone(constants.rotationPIDConstants.IZone);
        anglePID.enableContinuousInput(constants.rotationPIDContinuousConnections.getFirst(), constants.rotationPIDContinuousConnections.getSecond());

        xPID = new PIDController(
                constants.drivePIDConstants.P,
                constants.drivePIDConstants.I,
                constants.drivePIDConstants.D);
        xPID.setIZone(constants.drivePIDConstants.IZone);

        yPID = new PIDController(
                constants.drivePIDConstants.P,
                constants.drivePIDConstants.I,
                constants.drivePIDConstants.D);
        yPID.setIZone(constants.drivePIDConstants.IZone);
    }

    /**
     * Makes the swerve use PID to look at the given angle
     *
     * @param angle the angle to look at
     * @param roundToAngle the angle jumps to round to, for example 45 degrees will make it round
     *     the given angle to the nearest 0, 45, 90, 135... it rounds the angle only if the rounded
     *     angle is close enough to the given angle so for example if the given angle is 28 and the
     *     rounded angle is 45 it won't round. if you write 1 as the roundToAngle there will be no
     *     rounding, DON'T USE 0 (division by zero error)
     */
    public double lookAt(double angle, double roundToAngle) {
        double roundedAngle = Math.round(angle / roundToAngle) * roundToAngle;
        angle = Math.abs(roundedAngle - angle) <= roundToAngle / 3 ? roundedAngle : angle;

        return anglePID.calculate(RobotStateWithSwerve.getInstance().getGyroYaw().getRadians(), angle);
    }

    /**
     * Makes the swerve use PID to look according to the given direction
     *
     * @param direction - the direction vector to look
     * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
     *     the given angle (calculated from direction) to the nearest 0, 45, 90, 135... it rounds the
     *     angle only if the rounded angle is close enough to the given angle so for example if the
     *     given angle is 28 and the rounded angle is 45 it won't round. if you write 1 as the
     *     roundToAngle there will be no rounding, DON'T USE 0 (division by zero error)
     */
    public double lookAt(Translation2d direction, double roundToAngle) {
        if (!(direction.getX() == 0 && direction.getY() == 0))
            return lookAt(direction.getAngle().getRadians(), roundToAngle);

        return 0;
    }

    public double lookAtTarget(Pose2d target, Rotation2d offset) {
        Translation2d lookAtTranslation = RobotStateWithSwerve.getInstance().getTransform(target).getTranslation().rotateBy(offset);
        return lookAt(lookAtTranslation, 1);
    }

    public Translation2d pidTo(Translation2d target) {
        return new Translation2d(
            xPID.calculate(RobotStateWithSwerve.getInstance().getRobotPose().getX(), target.getX()),
            yPID.calculate(RobotStateWithSwerve.getInstance().getRobotPose().getY(), target.getY()));
    }

    public void setControl(ChassisSpeeds chassisSpeeds, boolean fieldRelative, String type) {
        if (type.equals(channel)) {
            lastInput = chassisSpeeds;
            Swerve.getInstance().drive(chassisSpeeds, fieldRelative);
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

    /**
     * Convert percent chassis speeds to m/s chassis speeds
     * @param percent the percent chassis speeds to convert
     * @return the m/s chassis speeds to give the swerve
     */
    public ChassisSpeeds fromPercent(ChassisSpeeds percent) {
        return new ChassisSpeeds(
            percent.vxMetersPerSecond * constants.swerveConstants.maxSpeed,
            percent.vyMetersPerSecond * constants.swerveConstants.maxSpeed,
            percent.omegaRadiansPerSecond * constants.swerveConstants.maxAngularVelocity
        );
    }

    public void periodic() {
        Swerve.getInstance().periodic();

        Logger.recordOutput("Swerve/Input", lastInput);
        Logger.recordOutput("Swerve/Channel", channel);
        Logger.recordOutput("Swerve/Previous State", previousChannel);
    }
}
