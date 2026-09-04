package frc.lib.NinjasLib.loggeddigitalinput;

import java.util.function.BooleanSupplier;

/**
 * {@link LoggedDigitalInputIO} implementation for simulation. Rather than reading real hardware,
 * the input's state is driven entirely by a caller-supplied {@link BooleanSupplier} (e.g. backed
 * by a mechanism simulation).
 */
public class LoggedDigitalInputIOSim implements LoggedDigitalInputIO {
    BooleanSupplier isOnSupplier;

    /**
     * Creates a simulated digital input whose state is read from {@code isOnSupplier}.
     *
     * @param isOnSupplier supplies the simulated on/off state on each {@link #update()} call
     */
    public LoggedDigitalInputIOSim(BooleanSupplier isOnSupplier) {
        this.isOnSupplier = isOnSupplier;
    }

    /**
     * No-op: simulated inputs have no hardware channel to configure, and are driven entirely by
     * the {@link BooleanSupplier} passed to the constructor.
     *
     * @param port     unused
     * @param inverted unused
     */
    @Override
    public void setup(int port, boolean inverted) {

    }

    /**
     * @return the current value of the constructor-supplied {@link BooleanSupplier}
     */
    @Override
    public boolean update() {
        return isOnSupplier.getAsBoolean();
    }
}
