package frc.lib.NinjasLib.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Hardware-agnostic interface for a single swerve module, following the AdvantageKit-style IO pattern:
 * implementations drive the physical (or simulated) drive/steer motors and populate a plain
 * {@link SwerveModuleIOInputs} data object each cycle. Implementations include
 * {@link SwerveModuleIOReal} and {@link SwerveModuleIOSim}.
 */
public interface SwerveModuleIO {
    /** Plain data holder for one cycle's worth of module state, populated by {@link #update()}. */
    class SwerveModuleIOInputs {
        /** Index of this module within the swerve drive, matching {@code SwerveModuleConstants#moduleNumber}. */
        public int moduleNumber;

        /** The last state commanded via {@link #setDesiredState}. */
        public SwerveModuleState desiredState = new SwerveModuleState();

        /** The module's current measured velocity and angle. */
        public SwerveModuleState state = new SwerveModuleState();

        /** The module's current measured distance traveled and angle, used for odometry. */
        public SwerveModulePosition position = new SwerveModulePosition();

        /** The steer angle reported by the module's absolute encoder (e.g. CANCoder). */
        public Rotation2d absolutePosition = Rotation2d.kZero;

        // Odometry Thread
        /** High-frequency steer angle samples collected since the last {@link #update()}, parallel to {@link #timestamps}. */
        public Rotation2d[] angles = new Rotation2d[0];

        /** High-frequency drive distance samples (meters) collected since the last {@link #update()}, parallel to {@link #timestamps}. */
        public double[] positions = new double[0];

        /** Timestamps (seconds) of the high-frequency samples in {@link #angles} and {@link #positions}. */
        public double[] timestamps = new double[0];
    }

    /**
     * Commands the module towards the given state, closing the loop on drive velocity unless
     * {@code isOpenLoop} is set.
     *
     * @param desiredState the target module speed and angle (not necessarily optimized yet;
     *     implementations are expected to optimize it against the current module angle)
     * @param isOpenLoop if {@code true}, drive the wheel by voltage/percent output instead of a
     *     velocity control loop
     * @param preventJittering if {@code true}, hold the last commanded angle instead of rotating the
     *     module when the requested speed is near zero, to avoid wheel jitter
     */
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean preventJittering);

    /**
     * Reads the current module state and returns a fresh {@link SwerveModuleIOInputs}. Should be
     * called once per robot loop cycle.
     *
     * @return the latest module readings
     */
    SwerveModuleIOInputs update();

    /**
     * Runs any per-cycle module control-loop updates that are not tied to a new {@link #setDesiredState}
     * call (e.g. re-applying the last commanded output). Should be called once per robot loop cycle.
     */
    void periodic();
}
