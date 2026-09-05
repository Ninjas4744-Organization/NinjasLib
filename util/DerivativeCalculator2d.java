package frc.lib.NinjasLib.util;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Computes a smoothed time-derivative (velocity) of a {@link Translation2d} value sampled over
 * time, e.g. to estimate a field-relative velocity from repeated pose measurements. This is
 * stateful: each call to {@link #calculate(Translation2d)} uses the value and timestamp from the
 * previous call, so it must be called once per sample (typically once per periodic loop) with a
 * consistent time source, and a single instance should not be shared across unrelated signals.
 */
public class DerivativeCalculator2d {
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;
    private Translation2d lastValue = new Translation2d();
    private double lastTimestamp;
    private boolean initialized = false;
    private Translation2d lastDerivative = new Translation2d();

    /**
     * @param averageWindow The number of samples to average over.
     * Higher = smoother but more "laggy". Try 5-10.
     */
    public DerivativeCalculator2d(int averageWindow) {
        this.xFilter = LinearFilter.movingAverage(averageWindow);
        this.yFilter = LinearFilter.movingAverage(averageWindow);
    }

    /**
     * Feeds in a new sample and returns the current smoothed derivative estimate. The first call
     * after construction (or after {@link #reset()}) has no prior sample to compare against, so it
     * returns a zero vector and only records {@code currentValue} as the baseline for the next call.
     *
     * @param currentValue The latest sampled value.
     * @return The smoothed rate of change of the value, per second.
     */
    public Translation2d calculate(Translation2d currentValue) {
        double currentTime = Timer.getFPGATimestamp();

        if (!initialized) {
            lastValue = currentValue;
            lastTimestamp = currentTime;
            initialized = true;
            return new Translation2d();
        }

        double dt = currentTime - lastTimestamp;

        // 1. Calculate the raw "noisy" derivative
        Translation2d rawDerivative = (dt > 0) ? (currentValue.minus(lastValue)).div(dt) : new Translation2d();

        // 2. Pass it through the filter to look at the "range of time"
        lastDerivative = new Translation2d(xFilter.calculate(rawDerivative.getX()), yFilter.calculate(rawDerivative.getY()));

        // 3. Update state
        lastValue = currentValue;
        lastTimestamp = currentTime;

        return lastDerivative;
    }

    /**
     * @return The most recently computed derivative, without taking a new sample. Same value
     *     {@link #calculate(Translation2d)} last returned.
     */
    public Translation2d get() {
        return lastDerivative;
    }

    /**
     * Clears all accumulated state (the moving-average filters and the last sample), so the next
     * call to {@link #calculate(Translation2d)} behaves as if this were a freshly constructed
     * calculator.
     */
    public void reset() {
        initialized = false;
        xFilter.reset();
        yFilter.reset();
    }
}
