package frc.lib.NinjasLib.loggeddigitalinput;

public interface LoggedDigitalInputIO {
    void setup(int port, boolean inverted);
    boolean update();
}
