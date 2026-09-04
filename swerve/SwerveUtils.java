package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Stateless math helpers for swerve module optimization and acceleration-limited velocity stepping. */
public class SwerveUtils {
    /**
     * Optimizes a module's desired state so it never has to rotate more than 90&deg; from its
     * current angle, flipping the wheel direction (and negating speed) instead when that's shorter.
     *
     * @param desiredState the target module state
     * @param currentAngle the module's current steer angle
     * @return an equivalent state reachable via the shortest rotation from {@code currentAngle}
     */
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

    /**
     * Steps {@code currentVelocity} one 20ms cycle towards {@code desiredVelocity}, capping
     * acceleration in the current direction of travel only (a speed-dependent cap: it tapers to
     * {@code 0} as {@code currentVelocity} approaches {@code maxVelocity}). Lateral (skid)
     * acceleration is left unbounded; see {@link #limitForwardAndSkidAcceleration} to also limit that.
     *
     * @param currentVelocity the robot's current field/robot-relative velocity
     * @param desiredVelocity the requested velocity to move towards
     * @param maxAcceleration the maximum forward acceleration, in m/s&sup2;, at zero speed
     * @param maxVelocity     the speed, in m/s, at which the forward acceleration cap reaches {@code 0}
     * @return the velocity to command for this cycle
     */
    public static Translation2d limitForwardAcceleration(Translation2d currentVelocity, Translation2d desiredVelocity, double maxAcceleration, double maxVelocity) {
        // Compute the velocity direction (normalize to get unit vector)
        Translation2d velocityDirection = currentVelocity.getNorm() > 0.05 ? currentVelocity.div(currentVelocity.getNorm()) : desiredVelocity.div(desiredVelocity.getNorm());

        // Compute max allowed acceleration in the current velocity direction
        double forwardMaxAccel = maxAcceleration * (1 - (currentVelocity.getNorm() / maxVelocity));

        // Compute wanted acceleration
        Translation2d wantedAccel = desiredVelocity.minus(currentVelocity).div(0.02);

        // Project acceleration onto the velocity direction
        double forwardAccel = wantedAccel.dot(velocityDirection);

        // Limit forward acceleration
        if (forwardAccel > forwardMaxAccel) {
            wantedAccel = wantedAccel.minus(velocityDirection.times(forwardAccel - forwardMaxAccel));
        }

        // Compute the next velocity
        return currentVelocity.plus(wantedAccel.times(0.02));
    }

    /**
     * Steps {@code currentVelocity} one 20ms cycle towards {@code desiredVelocity}, capping the
     * magnitude of the acceleration vector uniformly in every direction (unlike
     * {@link #limitForwardAcceleration}, which only limits acceleration along the current heading).
     *
     * @param currentVelocity    the robot's current field/robot-relative velocity
     * @param desiredVelocity    the requested velocity to move towards
     * @param maxSkidAcceleration the maximum acceleration magnitude, in m/s&sup2;
     * @return the velocity to command for this cycle
     */
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

    /**
     * Combines {@link #limitForwardAcceleration} and {@link #limitSkidAcceleration} in one step:
     * caps forward acceleration with a speed-dependent taper, then additionally caps the resulting
     * acceleration magnitude for skid protection. This is the limiter {@link Swerve#drive} uses
     * every cycle to smooth driver/autonomous input into an achievable velocity.
     *
     * @param currentVelocity        the robot's current field/robot-relative velocity
     * @param desiredVelocity        the requested velocity to move towards
     * @param maxForwardAcceleration the maximum forward acceleration, in m/s&sup2;, at zero speed
     * @param maxSkidAcceleration    the maximum overall acceleration magnitude, in m/s&sup2;
     * @param maxVelocity            the speed, in m/s, at which the forward acceleration cap reaches {@code 0}
     * @return the velocity to command for this cycle
     */
    public static Translation2d limitForwardAndSkidAcceleration(Translation2d currentVelocity, Translation2d desiredVelocity, double maxForwardAcceleration, double maxSkidAcceleration, double maxVelocity) {
        // Compute the velocity direction (normalize to get unit vector)
        double currentNorm = currentVelocity.getNorm();
        double desiredNorm = desiredVelocity.getNorm();

        Translation2d velocityDirection = new Translation2d();
        if (currentNorm > 0.05)
            velocityDirection = currentVelocity.div(currentNorm);
        else if (desiredNorm > 1e-6)
            velocityDirection = desiredVelocity.div(desiredNorm);

        // Compute max allowed acceleration in the current velocity direction
        double forwardMaxAccel = maxForwardAcceleration * (1 - MathUtil.clamp(currentVelocity.getNorm() / maxVelocity, 0, 1));

        // Compute wanted acceleration
        Translation2d wantedAccel = desiredVelocity.minus(currentVelocity).div(0.02);

        // Project acceleration onto the velocity direction
        double forwardAccel = wantedAccel.dot(velocityDirection);

        // Limit forward acceleration
        if (forwardAccel > forwardMaxAccel) {
            wantedAccel = wantedAccel.minus(velocityDirection.times(forwardAccel - forwardMaxAccel));
        }

        // If the magnitude of wantedAccel exceeds maxSkidAcceleration, scale it down
        if (wantedAccel.getNorm() > maxSkidAcceleration) {
            wantedAccel = wantedAccel.times(maxSkidAcceleration / wantedAccel.getNorm());
        }

        // Compute the next velocity
        return currentVelocity.plus(wantedAccel.times(0.02));
    }
}
