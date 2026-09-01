package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.wpilibj.XboxController;

public class LoggedCommandControllerIOXbox implements LoggedCommandControllerIO {
    private XboxController controller;

    public LoggedCommandControllerIOXbox(int port) {
        controller = new XboxController(port);
    }

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
