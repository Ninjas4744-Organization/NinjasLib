package frc.lib.NinjasLib.loggedcontroller;

import org.littletonrobotics.junction.AutoLog;

public interface LoggedCommandControllerIO {
    @AutoLog
    class LoggedCommandControllerIOInputs {
        // Face buttons
        public boolean cross;
        public boolean circle;
        public boolean square;
        public boolean triangle;

        // D-Pad (POV)
        public boolean povUp;
        public boolean povDown;
        public boolean povLeft;
        public boolean povRight;

        // Stick buttons
        public boolean L3;
        public boolean R3;

        // Bumpers
        public boolean L1;
        public boolean R1;

        // Triggers (as digital buttons)
        public boolean L2;
        public boolean R2;

        // Options / System
        public boolean create;    // left of touchpad
        public boolean options;   // right of touchpad
        public boolean ps;        // PlayStation logo button
        public boolean touchpad;  // touchpad press

        // Axes (joysticks and triggers)
        public double leftX;
        public double leftY;
        public double rightX;
        public double rightY;
        public double L2Axis;
        public double R2Axis;
    }

    default void updateInputs(LoggedCommandControllerIOInputsAutoLogged inputs) {
    }
}
