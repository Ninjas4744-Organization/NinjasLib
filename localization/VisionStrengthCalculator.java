package frc.lib.NinjasLib.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.NinjasLib.localization.vision.VisionOutput;

import java.util.function.DoubleSupplier;

/**
 * A pluggable per-axis trust ("Kalman strength") calculator for vision measurements, injected into
 * {@link RobotPose} and passed on to {@link NinjasPoseTracker#setVisionMeasurementStrength} for
 * measurements that pass the configured {@link VisionFiltersCalculator}. This is what determines how
 * hard a given vision measurement pulls the pose estimate towards it versus trusting odometry.
 */
@FunctionalInterface
public interface VisionStrengthCalculator {
    /**
     * Computes the per-axis trust to apply to a vision measurement.
     *
     * @param visionOutput The camera's vision estimate for this cycle.
     * @return Per-axis trust in [0, 1]: x (meters), y (meters), theta (radians).
     */
    Matrix<N3, N1> calculate(VisionOutput visionOutput);

    /** A fixed, distance- and drift-independent trust of {@code (0.05, 0.05, 0.025)} for every measurement. */
    VisionStrengthCalculator kDefault = estimation -> VecBuilder.fill(0.05, 0.05, 0.025);

    /**
     * Builds a strength calculator whose trust in a vision measurement decreases with the square of
     * the distance to the closest visible tag, and increases as accumulated odometry drift grows (drift
     * is floored by {@code epsilon} before being raised to a power that grows slowly below 1 and
     * quickly above 1, so trust in vision ramps up faster once odometry has drifted noticeably).
     *
     * @param factor Larger values reduce the effect of distance, yielding higher overall trust.
     * @param epsilon A small floor added to {@code odometryDrift} to avoid degenerate behavior near
     *     zero drift.
     * @param rotationFactor Multiplier applied to the x/y trust to get the rotation (theta) trust.
     * @param odometryDrift Supplies a current estimate of accumulated odometry drift/uncertainty.
     * @return A strength calculator combining target distance and odometry drift.
     */
    static VisionStrengthCalculator ninjasFunction(double factor, double epsilon, double rotationFactor, DoubleSupplier odometryDrift) {
        return estimation -> {
            double distancePart = Math.pow(estimation.closestTargetDist, 2);

            double driftPart = Math.max(Math.pow(odometryDrift.getAsDouble() + epsilon, 0.5), Math.pow(odometryDrift.getAsDouble() + epsilon, 2));

            double visionStrength = 1 / (1 + distancePart / factor / driftPart);

            return VecBuilder.fill(visionStrength, visionStrength, visionStrength * rotationFactor);
        };
    }

    /**
     * Same as {@link #ninjasFunction(double, double, double, DoubleSupplier)}, using the library's
     * default tuning ({@code factor=1.5}, {@code epsilon=0.05}, {@code rotationFactor=0.5}).
     *
     * @param odometryDrift Supplies a current estimate of accumulated odometry drift/uncertainty.
     * @return A strength calculator combining target distance and odometry drift, with default tuning.
     */
    static VisionStrengthCalculator ninjasFunction(DoubleSupplier odometryDrift) {
        return ninjasFunction(1.5, 0.05, 0.5, odometryDrift);
    }
}
