package frc.lib.NinjasLib.loggeddigitalinput;

import org.littletonrobotics.junction.AutoLog;

public interface LoggedDigitalInputIO {
    @AutoLog
    class LoggedDigitalInputs {
        boolean IsOn;
    }

    default void updateInputs(LoggedDigitalInputsAutoLogged inputs) {}

    default void setup(int port) {}
}
