package frc.lib.NinjasLib.util;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

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

    public Translation2d get() {
        return lastDerivative;
    }

    public void reset() {
        initialized = false;
        xFilter.reset();
        yFilter.reset();
    }
}
