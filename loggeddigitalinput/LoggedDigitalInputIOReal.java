package frc.lib.NinjasLib.loggeddigitalinput;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * {@link LoggedDigitalInputIO} implementation for a real robot, backed by a physical
 * {@link DigitalInput} channel.
 */
public class LoggedDigitalInputIOReal implements LoggedDigitalInputIO {
    private DigitalInput input;
    private boolean inverted;

    /**
     * Creates the {@link DigitalInput} on the given port.
     *
     * @param port     the digital I/O port the input is wired to
     * @param inverted whether the raw hardware reading should be inverted before use
     */
    @Override
    public void setup(int port, boolean inverted) {
        input = new DigitalInput(port);
        this.inverted = inverted;
    }

    /**
     * Reads the current state of the digital input, XOR-ed with {@code inverted}.
     *
     * @return {@code true} if the input is on (triggered)
     */
    @Override
    public boolean update() {
        return input.get() ^ inverted;
    }
}
