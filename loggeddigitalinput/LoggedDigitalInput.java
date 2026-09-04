package frc.lib.NinjasLib.loggeddigitalinput;

import frc.lib.NinjasLib.util.NinjasLogger;

/**
 * Wraps a {@link LoggedDigitalInputIO} (e.g. a limit switch or beam break, real or simulated) to
 * add AdvantageKit logging and an enable/disable fallback. When disabled, the digital input's
 * hardware is never touched and {@link #get()} always returns a fixed {@code disabledValue}
 * instead — useful for optional sensors that may not be wired on a given robot. Call
 * {@link #periodic()} once per loop to refresh and log the current state.
 */
public class LoggedDigitalInput {
    private LoggedDigitalInputIO io;
    private boolean isOn;
    private boolean enabled;
    private boolean disabledValue;
    private String name;

    /**
     * Creates a logged digital input. If {@code enabled} is {@code true}, {@code io} is
     * immediately configured via {@link LoggedDigitalInputIO#setup(int, boolean)}; otherwise
     * {@code io} is never used and {@link #get()} always returns {@code disabledValue}.
     *
     * @param name          the AdvantageKit log key this input's state is logged under
     * @param port          the digital I/O port the input is wired to, passed through to {@code io}
     * @param enabled       whether this input is physically present and should be polled
     * @param disabledValue the value {@link #get()} returns when {@code enabled} is {@code false}
     * @param inverted      whether the raw hardware reading should be inverted, passed through to {@code io}
     * @param io            the hardware-specific IO implementation (e.g. {@link LoggedDigitalInputIOReal}
     *                      or {@link LoggedDigitalInputIOSim}) to poll each cycle
     */
    public LoggedDigitalInput(String name, int port, boolean enabled, boolean disabledValue, boolean inverted, LoggedDigitalInputIO io) {
        this.enabled = enabled;
        this.disabledValue = disabledValue;
        this.name = name;

        if(enabled){
            this.io = io;
            io.setup(port, inverted);
        }
    }

    /**
     * @return the last-read state of the input if enabled, or the constructor-supplied
     *     {@code disabledValue} if this input is disabled
     */
    public boolean get() {
        return enabled ? isOn : disabledValue;
    }

    /**
     * If enabled, polls the underlying {@link LoggedDigitalInputIO} and logs the result to
     * AdvantageKit under this input's name. Must be called once per robot loop for {@link #get()}
     * to reflect the current hardware state; a no-op when this input is disabled.
     */
    public void periodic() {
        if(enabled){
            isOn = io.update();
            NinjasLogger.log(name, isOn);
        }
    }
}
