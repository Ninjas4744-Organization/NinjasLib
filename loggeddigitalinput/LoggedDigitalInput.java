package frc.lib.NinjasLib.loggeddigitalinput;

import frc.lib.NinjasLib.NinjasLogger;

public class LoggedDigitalInput {
    private LoggedDigitalInputIO io;
    private boolean isOn;
    private boolean enabled;
    private boolean disabledValue;
    private String name;

    public LoggedDigitalInput(String name, int port, boolean enabled, boolean disabledValue, boolean inverted, LoggedDigitalInputIO io) {
        this.enabled = enabled;
        this.disabledValue = disabledValue;
        this.name = name;

        if(enabled){
            this.io = io;
            io.setup(port, inverted);
        }
    }

    public boolean get() {
        return enabled ? isOn : disabledValue;
    }

    public void periodic() {
        if(enabled){
            isOn = io.update();
            NinjasLogger.log(name, isOn);
        }
    }
}
