package frc.lib.NinjasLib.loggeddigitalinput;

import org.littletonrobotics.junction.Logger;

public class LoggedDigitalInput {
    private LoggedDigitalInputIO io;
    private LoggedDigitalInputsAutoLogged inputs = new LoggedDigitalInputsAutoLogged();
    private boolean enabled;
    private boolean disabledValue;
    private String name;

    public LoggedDigitalInput(String name, int port, boolean enabled, boolean disabledValue, LoggedDigitalInputIO io) {
        this.enabled = enabled;
        this.disabledValue = disabledValue;
        this.name = name;

        if(enabled){
            this.io = io;
            io.setup(port);
        }
    }

    public boolean get() {
        return enabled ? inputs.IsOn : disabledValue;
    }

    public void periodic() {
        if(enabled){
            io.updateInputs(inputs);
            Logger.processInputs(name, inputs);
        }
    }
}
