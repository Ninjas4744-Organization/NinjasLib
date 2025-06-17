package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
    public static SwerveModuleState optimizeModuleState(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double currentDegrees = currentAngle.getDegrees();
        double targetDegrees = desiredState.angle.getDegrees();

        double delta = targetDegrees - currentDegrees;
        delta = (delta + 360) % 360;  // Normalize delta to [0, 360)

        if (delta > 180) delta -= 360;  // Adjust to [-180, 180)

        if (Math.abs(delta) > 90) {
            targetDegrees += delta > 0 ? -180 : 180;
            desiredState = new SwerveModuleState(-desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(targetDegrees));
        }

        return new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromDegrees((targetDegrees + 360) % 360));
    }

    public static Translation2d limitForwardAcceleration(Translation2d currentVelocity, Translation2d desiredVelocity, double maxAcceleration, double maxVelocity) {
        // Compute the velocity direction (normalize to get unit vector)
        Translation2d velocityDirection = currentVelocity.getNorm() > 0 ? currentVelocity.div(currentVelocity.getNorm()) : new Translation2d();

        // Compute max allowed acceleration in the current velocity direction
        double forwardMaxAccel = maxAcceleration * (1 - (currentVelocity.getNorm() / maxVelocity));

        // Compute wanted acceleration
        Translation2d wantedAccel = desiredVelocity.minus(currentVelocity).div(0.02);

        // Project acceleration onto the velocity direction
        double forwardAccel = wantedAccel.toVector().dot(velocityDirection.toVector());

        // Limit forward acceleration
        if (forwardAccel > forwardMaxAccel) {
            wantedAccel = wantedAccel.minus(velocityDirection.times(forwardAccel - forwardMaxAccel));
        }

        // Compute the next velocity
        return currentVelocity.plus(wantedAccel.times(0.02));
    }

    public static Translation2d limitSkidAcceleration(Translation2d currentVelocity, Translation2d desiredVelocity, double maxSkidAcceleration) {
        // Compute the wanted acceleration
        Translation2d wantedAccel = desiredVelocity.minus(currentVelocity).div(0.02);

        // If the magnitude of wantedAccel exceeds maxSkidAcceleration, scale it down
        if (wantedAccel.getNorm() > maxSkidAcceleration) {
            wantedAccel = wantedAccel.times(maxSkidAcceleration / wantedAccel.getNorm());
        }

        // Compute the next velocity
        return currentVelocity.plus(wantedAccel.times(0.02));
    }
}
