package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.wpilibj.PS5Controller;

/**
 * {@link LoggedCommandControllerIO} implementation for a PlayStation 5 (DualSense) controller,
 * backed by a {@link PS5Controller}. The Xbox-named fields ({@code A}/{@code B}/{@code X}/
 * {@code Y}, {@code back}/{@code start}) are filled in from their PS5 equivalents so this
 * controller can be swapped in wherever a {@link LoggedCommandControllerIOXbox} is expected.
 */
public class LoggedCommandControllerIOPS5 implements LoggedCommandControllerIO {
    private PS5Controller controller;

    /**
     * Creates a PS5 controller IO bound to the given driver station port.
     *
     * @param port the driver station USB port the controller is plugged into
     */
    public LoggedCommandControllerIOPS5(int port) {
        controller = new PS5Controller(port);
    }

    /**
     * Reads the current state of every button, POV, and axis from the underlying
     * {@link PS5Controller}.
     *
     * @return the latest {@link LoggedCommandControllerIOInputs}
     */
    @Override
    public LoggedCommandControllerIOInputs update() {
        LoggedCommandControllerIOInputs inputs = new LoggedCommandControllerIOInputs();

        // Face buttons
        inputs.cross = controller.getCrossButton();
        inputs.circle = controller.getCircleButton();
        inputs.square = controller.getSquareButton();
        inputs.triangle = controller.getTriangleButton();

        // Face buttons XBOX
        inputs.A = controller.getCrossButton();
        inputs.B = controller.getCircleButton();
        inputs.X = controller.getSquareButton();
        inputs.Y = controller.getTriangleButton();

        // D-Pad (POV)
        inputs.povUp = controller.getPOV() == 0;
        inputs.povRight = controller.getPOV() == 90;
        inputs.povDown = controller.getPOV() == 180;
        inputs.povLeft = controller.getPOV() == 270;

        // Stick buttons
        inputs.L3 = controller.getL3Button();
        inputs.R3 = controller.getR3Button();

        // Bumpers
        inputs.L1 = controller.getL1Button();
        inputs.R1 = controller.getR1Button();

        // Triggers (as digital buttons)
        inputs.L2 = controller.getL2Button();
        inputs.R2 = controller.getR2Button();

        // Options / System
        inputs.create = controller.getCreateButton();   // left of touchpad
        inputs.options = controller.getOptionsButton(); // right of touchpad
        inputs.ps = controller.getPSButton();
        inputs.touchpad = controller.getTouchpadButton();

        // Options / System XBOX
        inputs.back = controller.getCreateButton();   // left of touchpad
        inputs.start = controller.getOptionsButton(); // right of touchpad

        // Axes (joysticks and triggers)
        inputs.leftX = controller.getLeftX();
        inputs.leftY = controller.getLeftY();
        inputs.rightX = controller.getRightX();
        inputs.rightY = controller.getRightY();
        inputs.L2Axis = controller.getL2Axis();
        inputs.R2Axis = controller.getR2Axis();

        return inputs;
    }
}
