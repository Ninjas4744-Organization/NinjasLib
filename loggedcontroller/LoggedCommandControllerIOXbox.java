package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.wpilibj.XboxController;

/**
 * {@link LoggedCommandControllerIO} implementation for an Xbox controller, backed by an
 * {@link XboxController}. The PlayStation-named fields ({@code cross}/{@code circle}/
 * {@code square}/{@code triangle}, {@code create}/{@code options}) are filled in from their Xbox
 * equivalents so this controller can be swapped in wherever a {@link LoggedCommandControllerIOPS5}
 * is expected. {@code ps} and {@code touchpad} have no Xbox equivalent and are always
 * {@code false}; the trigger axes are treated as pressed (as {@code L2}/{@code R2}) once they
 * exceed {@code 0.7}.
 */
public class LoggedCommandControllerIOXbox implements LoggedCommandControllerIO {
    private XboxController controller;

    /**
     * Creates an Xbox controller IO bound to the given driver station port.
     *
     * @param port the driver station USB port the controller is plugged into
     */
    public LoggedCommandControllerIOXbox(int port) {
        controller = new XboxController(port);
    }

    /**
     * Reads the current state of every button, POV, and axis from the underlying
     * {@link XboxController}.
     *
     * @return the latest {@link LoggedCommandControllerIOInputs}
     */
    @Override
    public LoggedCommandControllerIOInputs update() {
        LoggedCommandControllerIOInputs inputs = new LoggedCommandControllerIOInputs();

        // Face buttons
        inputs.cross = controller.getAButton();
        inputs.circle = controller.getBButton();
        inputs.square = controller.getXButton();
        inputs.triangle = controller.getYButton();

        // Face buttons XBOX
        inputs.A = controller.getAButton();
        inputs.B = controller.getBButton();
        inputs.X = controller.getXButton();
        inputs.Y = controller.getYButton();

        // D-Pad (POV)
        inputs.povUp = controller.getPOV() == 0;
        inputs.povRight = controller.getPOV() == 90;
        inputs.povDown = controller.getPOV() == 180;
        inputs.povLeft = controller.getPOV() == 270;

        // Stick buttons
        inputs.L3 = controller.getLeftStickButton();
        inputs.R3 = controller.getRightStickButton();

        // Bumpers
        inputs.L1 = controller.getLeftBumperButton();
        inputs.R1 = controller.getRightBumperButton();

        // Triggers (as digital buttons)
        inputs.L2 = controller.getLeftTriggerAxis() > 0.7;
        inputs.R2 = controller.getRightTriggerAxis() > 0.7;

        // Options / System
        inputs.create = controller.getBackButton();   // left of touchpad
        inputs.options = controller.getStartButton(); // right of touchpad
        inputs.ps = false;
        inputs.touchpad = false;

        // Options / System XBOX
        inputs.back = controller.getBackButton();   // left of touchpad
        inputs.start = controller.getStartButton(); // right of touchpad

        // Axes (joysticks and triggers)
        inputs.leftX = controller.getLeftX();
        inputs.leftY = controller.getLeftY();
        inputs.rightX = controller.getRightX();
        inputs.rightY = controller.getRightY();
        inputs.L2Axis = controller.getLeftTriggerAxis();
        inputs.R2Axis = controller.getRightTriggerAxis();
        
        return inputs;
    }
}
