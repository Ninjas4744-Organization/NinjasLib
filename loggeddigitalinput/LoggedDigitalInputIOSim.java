package frc.lib.NinjasLib.loggeddigitalinput;

import java.util.function.BooleanSupplier;

public class LoggedDigitalInputIOSim implements LoggedDigitalInputIO{
    BooleanSupplier isOnSupplier;

    public LoggedDigitalInputIOSim(BooleanSupplier isOnSupplier) {
        this.isOnSupplier = isOnSupplier;
    }

    @Override
    public void updateInputs(LoggedDigitalInputsAutoLogged inputs) {
        inputs.IsOn = isOnSupplier.getAsBoolean();
    }
}
