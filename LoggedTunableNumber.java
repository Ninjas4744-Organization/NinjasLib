package frc.lib.NinjasLib;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import frc.robot.RobotState;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
@SuppressWarnings("unused")
public class LoggedTunableNumber implements DoubleSupplier {
    private static final String tableKey = "/Tuning";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    private final boolean tuningMode;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     * @param isTuning Get whether the value is in tuning mode
     */
    public LoggedTunableNumber(String dashboardKey, boolean isTuning) {
        this.key = tableKey + "/" + dashboardKey;
        this.tuningMode = isTuning;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param isTuning Get whether the value is in tuning mode
     * @param defaultValue Default value
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue, boolean isTuning) {
        this(dashboardKey, isTuning);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (tuningMode) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return tuningMode ? dashboardNumber.get() : defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id) {
        if (!tuningMode)
            return false;

        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable
     *     numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(
            int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}