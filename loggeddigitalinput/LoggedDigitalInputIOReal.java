package frc.lib.NinjasLib.loggeddigitalinput;

import edu.wpi.first.wpilibj.DigitalInput;

public class LoggedDigitalInputIOReal implements LoggedDigitalInputIO{
    private DigitalInput input;

    @Override
    public void setup(int port) {
        input = new DigitalInput(port);
    }

    @Override
    public void updateInputs(LoggedDigitalInputsAutoLogged inputs) {
        inputs.IsOn = input.get();
    }
}
