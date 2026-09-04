package frc.lib.NinjasLib.loggeddigitalinput;

/**
 * Hardware-agnostic IO interface for a single digital input (e.g. a limit switch or beam break).
 * Implemented by {@link LoggedDigitalInputIOReal} for a physical {@code DigitalInput} channel and
 * by {@link LoggedDigitalInputIOSim} for simulation, and wrapped by {@link LoggedDigitalInput}
 * which adds AdvantageKit logging and an enabled/disabled fallback.
 */
public interface LoggedDigitalInputIO {
    /**
     * Configures the underlying hardware channel. Called once, before any call to
     * {@link #update()}.
     *
     * @param port     the digital I/O port the input is wired to
     * @param inverted whether the raw hardware reading should be inverted before use
     */
    void setup(int port, boolean inverted);

    /**
     * Reads the current state of the digital input.
     *
     * @return {@code true} if the input is on (triggered), accounting for the {@code inverted}
     *     flag passed to {@link #setup(int, boolean)}
     */
    boolean update();
}
