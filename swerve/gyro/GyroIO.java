package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Hardware-agnostic interface for a swerve gyro, following the AdvantageKit-style IO pattern:
 * implementations read the physical (or simulated) sensor and populate a plain {@link GyroIOInputs}
 * data object each cycle, so higher-level code (see {@link Gyro}) never touches vendor APIs directly.
 * Implementations include {@link GyroIOPigeon2}, {@link GyroIONavX}, and {@link GyroIOSim}.
 */
public interface GyroIO {
    /** Plain data holder for one cycle's worth of gyro readings, populated by {@link #update()}. */
    class GyroIOInputs {
        /** Current gyro yaw, unaffected by {@link #resetGyroYaw}. */
        public Rotation2d yaw = Rotation2d.kZero;

        /** Current gyro yaw with the user-applied offset from {@link #resetGyroYaw} applied. */
        public Rotation2d yawOffsetted = Rotation2d.kZero;

        /** Current gyro pitch. */
        public Rotation2d pitch = Rotation2d.kZero;

        /** Current gyro roll. */
        public Rotation2d roll = Rotation2d.kZero;

        /** Acceleration along the robot's X axis, in m/s^2. */
        public double accelerationX;

        /** Acceleration along the robot's Y axis, in m/s^2. */
        public double accelerationY;

        /** Acceleration along the robot's Z axis, in m/s^2. */
        public double accelerationZ;

        /** Timestamps (seconds) of high-frequency yaw samples collected since the last {@link #update()}, for odometry. */
        public double[] odometryYawTimestamps = new double[] {};

        /** High-frequency, offset-adjusted yaw samples collected since the last {@link #update()}, parallel to {@link #odometryYawTimestamps}. */
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    /**
     * Reads the current sensor state and returns a fresh {@link GyroIOInputs}. Should be called once
     * per robot loop cycle, typically from {@link Gyro#periodic()}.
     *
     * @return the latest gyro readings
     */
    GyroIOInputs update();

    /**
     * Re-zeroes (or re-references) the gyro so that {@link GyroIOInputs#yawOffsetted} reports
     * {@code yaw} at the current physical orientation, without disturbing the raw
     * {@link GyroIOInputs#yaw} reading.
     *
     * @param yaw the yaw the gyro should report at its current orientation
     */
    void resetGyroYaw(Rotation2d yaw);
}
