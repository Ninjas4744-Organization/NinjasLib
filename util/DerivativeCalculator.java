package frc.lib.NinjasLib.util;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

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

    public double get() {
        return lastDerivative;
    }

    public void reset() {
        initialized = false;
        lowPassFilter.reset();
    }
}
