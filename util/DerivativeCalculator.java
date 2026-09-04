package frc.lib.NinjasLib.util;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Computes a smoothed time-derivative (rate of change) of a scalar value sampled over time, e.g.
 * to estimate velocity from repeated position measurements. This is stateful: each call to
 * {@link #calculate(double)} uses the value and timestamp from the previous call, so it must be
 * called once per sample (typically once per periodic loop) with a consistent time source, and a
 * single instance should not be shared across unrelated signals.
 */
public class DerivativeCalculator {
    private final LinearFilter lowPassFilter;
    private double lastValue;
    private double lastTimestamp;
    private boolean initialized = false;
    private double lastDerivative;

    /**
     * @param averageWindow The number of samples to average over.
     * Higher = smoother but more "laggy". Try 5-10.
     */
    public DerivativeCalculator(int averageWindow) {
        this.lowPassFilter = LinearFilter.movingAverage(averageWindow);
    }

    /**
     * Feeds in a new sample and returns the current smoothed derivative estimate. The first call
     * after construction (or after {@link #reset()}) has no prior sample to compare against, so it
     * returns {@code 0.0} and only records {@code currentValue} as the baseline for the next call.
     *
     * @param currentValue The latest sampled value.
     * @return The smoothed rate of change of the value, per second.
     */
    public double calculate(double currentValue) {
        double currentTime = Timer.getFPGATimestamp();

        if (!initialized) {
            lastValue = currentValue;
            lastTimestamp = currentTime;
            initialized = true;
            return 0.0;
        }

        double dt = currentTime - lastTimestamp;

        // 1. Calculate the raw "noisy" derivative
        double rawDerivative = (dt > 0) ? (currentValue - lastValue) / dt : 0;

        // 2. Pass it through the filter to look at the "range of time"
        lastDerivative = lowPassFilter.calculate(rawDerivative);

        // 3. Update state
        lastValue = currentValue;
        lastTimestamp = currentTime;

        return lastDerivative;
    }

    /**
     * @return The most recently computed derivative, without taking a new sample. Same value
     *     {@link #calculate(double)} last returned.
     */
    public double get() {
        return lastDerivative;
    }

    /**
     * Clears all accumulated state (the moving-average filter and the last sample), so the next
     * call to {@link #calculate(double)} behaves as if this were a freshly constructed calculator.
     */
    public void reset() {
        initialized = false;
        lowPassFilter.reset();
    }
}
