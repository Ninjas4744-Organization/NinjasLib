package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.dataclasses.SwerveControllerConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveController {
    private final PIDController anglePID;
    private final PIDController xPID;
    private final PIDController yPID;
    private final SwerveControllerConstants constants;

    private SwerveInput lastInput;
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
        lastInput = new SwerveInput();

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

    public void setControl(SwerveInput input, String channel) {
        if (channel.equals(this.channel)) {
//            if (constants.enableRotationPIDCorrection) {
////                double omegaRadiansPerSecond = anglePID.calculate(RobotStateWithSwerve.getInstance().getGyroYaw().getRadians(), targetAngle.getRadians());
////                lastInput = new SwerveInput(new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, omegaRadiansPerSecond), fieldRelative);
////                Swerve.getInstance().drive(lastInput.getChassisSpeeds(), fieldRelative);
//
////                targetAngle = targetAngle.plus(Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * 0.02));
//                if (Math.abs(lastInput.getO()) < 0.1) {
//                    lastInput = new SwerveInput(chassisSpeeds, fieldRelative);
//                    Swerve.getInstance().drive(new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, rotationCorrectionLastInput), lastInput.isFieldRelative());
//                } else {
//                    targetAngle = Rotation2d.fromRadians((targetAngle.getRadians() + chassisSpeeds.omegaRadiansPerSecond * 0.02 * 20 + RobotStateWithSwerve.getInstance().getGyroYaw().getRadians()) / 2);
//                    lastInput = new SwerveInput(chassisSpeeds, fieldRelative);
//                    Swerve.getInstance().drive(lastInput.getChassisSpeeds(), lastInput.isFieldRelative());
//                }
//            } else {
//                lastInput = new SwerveInput(chassisSpeeds, fieldRelative);
//                Swerve.getInstance().drive(lastInput.getChassisSpeeds(), lastInput.isFieldRelative());
//            }

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

    /**
     * Convert percent chassis speeds to m/s chassis speeds
     * @param percent the percent chassis speeds to convert
     * @return the m/s chassis speeds to give the swerve
     */
    public SwerveInput fromPercent(SwerveInput percent) {
        return new SwerveInput(
            percent.vxMetersPerSecond * constants.swerveConstants.maxSpeed,
            percent.vyMetersPerSecond * constants.swerveConstants.maxSpeed,
            percent.omegaRadiansPerSecond * constants.swerveConstants.maxAngularVelocity,
            percent.isFieldRelative()
        );
    }

    public void periodic() {
//        if (constants.enableRotationPIDCorrection) {
//            if (Math.abs(lastInput.getO()) < 0.1) {
//                rotationCorrectionLastInput = anglePID.calculate(RobotStateWithSwerve.getInstance().getGyroYaw().getRadians(), targetAngle.getRadians());
//                Swerve.getInstance().drive(new ChassisSpeeds(lastInput.getVx(), lastInput.getVy(), rotationCorrectionLastInput), lastInput.isFieldRelative());
//            }
//
//            Logger.recordOutput("Swerve/Target Angle", targetAngle);
//            Pose2d robotPose = RobotStateWithSwerve.getInstance().getRobotPose();
//            Logger.recordOutput("Swerve/Target Angle Pose", new Pose2d(robotPose.getTranslation(), targetAngle));
//        }
        Swerve.getInstance().periodic();

        Logger.recordOutput("Swerve/Input", lastInput);
        Logger.recordOutput("Swerve/Channel", channel);
        Logger.recordOutput("Swerve/Previous Channel", previousChannel);
    }
}
