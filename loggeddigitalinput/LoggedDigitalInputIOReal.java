package frc.lib.NinjasLib.loggeddigitalinput;

import edu.wpi.first.wpilibj.DigitalInput;

public class LoggedDigitalInputIOReal implements LoggedDigitalInputIO{
    private DigitalInput input;
    private boolean inverted;

    @Override
    public void setup(int port, boolean inverted) {
        input = new DigitalInput(port);
        this.inverted = inverted;
    }

    @Override
    public void updateInputs(LoggedDigitalInputsAutoLogged inputs) {
        inputs.IsOn = input.get() ^ inverted;
    }
}
