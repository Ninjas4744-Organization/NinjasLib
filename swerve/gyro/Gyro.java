package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.NinjasLib.util.NinjasLogger;

/**
 * Wraps a {@link GyroIO} implementation to give the rest of the robot code a single, hardware-agnostic
 * gyro API: it caches the latest {@link GyroIO.GyroIOInputs} and exposes typed getters, and logs
 * every field via {@link NinjasLogger} on each {@link #periodic()} call.
 */
public class Gyro {
    private GyroIO io;
    private GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();

    /**
     * @param io the hardware-specific (or simulated) implementation backing this gyro
     */
    public Gyro(GyroIO io) {
        this.io = io;
    }

    /** @return the raw gyro yaw, unaffected by {@link #resetYaw} */
    public Rotation2d getYaw() {
        return inputs.yaw;
    }

    /** @return the gyro yaw with the user-applied {@link #resetYaw} offset applied */
    public Rotation2d getYawOffsetted() {
        return inputs.yawOffsetted;
    }

    /** @return the current gyro pitch */
    public Rotation2d getPitch() {
        return inputs.pitch;
    }

    /** @return the current gyro roll */
    public Rotation2d getRoll() {
        return inputs.roll;
    }

    /** @return acceleration along the robot's X axis, in m/s^2 */
    public double getAccelerationX() {
        return inputs.accelerationX;
    }

    /** @return acceleration along the robot's Y axis, in m/s^2 */
    public double getAccelerationY() {
        return inputs.accelerationY;
    }

    /** @return acceleration along the robot's Z axis, in m/s^2 */
    public double getAccelerationZ() {
        return inputs.accelerationZ;
    }

    /** @return timestamps of the high-frequency yaw samples collected since the last {@link #periodic()} */
    public double[] getOdometryYawTimestamps() {
        return inputs.odometryYawTimestamps;
    }

    /** @return high-frequency, offset-adjusted yaw samples, parallel to {@link #getOdometryYawTimestamps()} */
    public Rotation2d[] getOdometryYawPositions() {
        return inputs.odometryYawPositions;
    }

    /**
     * Re-zeroes (or re-references) the gyro so it reports {@code yaw} at its current physical
     * orientation, e.g. to sync the gyro with a known field heading.
     *
     * @param yaw the yaw the gyro should report at its current orientation
     */
    public void resetYaw(Rotation2d yaw) {
        io.resetGyroYaw(yaw);
    }

    /**
     * Refreshes the cached gyro readings from the underlying {@link GyroIO} and logs them. Must be
     * called once per robot loop cycle for the getters on this class to return current data.
     */
    public void periodic() {
        inputs = io.update();
        NinjasLogger.log("Swerve/Gyro/Yaw", inputs.yaw);
        NinjasLogger.log("Swerve/Gyro/Yaw Offsetted", inputs.yawOffsetted);
        NinjasLogger.log("Swerve/Gyro/Pitch", inputs.pitch);
        NinjasLogger.log("Swerve/Gyro/Roll", inputs.roll);
        NinjasLogger.log("Swerve/Gyro/Acceleration", new Translation3d(inputs.accelerationX, inputs.accelerationY, inputs.accelerationZ));
    }
}
