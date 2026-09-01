package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.NinjasLogger;

public class LoggedCommandController {
    private LoggedCommandControllerIO.LoggedCommandControllerIOInputs inputs = new LoggedCommandControllerIO.LoggedCommandControllerIOInputs();
    private LoggedCommandControllerIO io;
    private String name;

    public LoggedCommandController(String name, LoggedCommandControllerIO io) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        inputs = io.update();

        // Face Buttons (PS & Xbox)
        NinjasLogger.log(name + "/cross", inputs.cross);
        NinjasLogger.log(name + "/circle", inputs.circle);
        NinjasLogger.log(name + "/square", inputs.square);
        NinjasLogger.log(name + "/triangle", inputs.triangle);
        NinjasLogger.log(name + "/A", inputs.A);
        NinjasLogger.log(name + "/B", inputs.B);
        NinjasLogger.log(name + "/X", inputs.X);
        NinjasLogger.log(name + "/Y", inputs.Y);

        // D-Pad (POV)
        NinjasLogger.log(name + "/povUp", inputs.povUp);
        NinjasLogger.log(name + "/povDown", inputs.povDown);
        NinjasLogger.log(name + "/povLeft", inputs.povLeft);
        NinjasLogger.log(name + "/povRight", inputs.povRight);

        // Stick Buttons & Bumpers
        NinjasLogger.log(name + "/L3", inputs.L3);
        NinjasLogger.log(name + "/R3", inputs.R3);
        NinjasLogger.log(name + "/L1", inputs.L1);
        NinjasLogger.log(name + "/R1", inputs.R1);

        // Triggers (Digital)
        NinjasLogger.log(name + "/L2", inputs.L2);
        NinjasLogger.log(name + "/R2", inputs.R2);

        // System Buttons
        NinjasLogger.log(name + "/create", inputs.create);
        NinjasLogger.log(name + "/options", inputs.options);
        NinjasLogger.log(name + "/ps", inputs.ps);
        NinjasLogger.log(name + "/touchpad", inputs.touchpad);
        NinjasLogger.log(name + "/back", inputs.back);
        NinjasLogger.log(name + "/start", inputs.start);

        // Axes
        NinjasLogger.log(name + "/leftX", inputs.leftX);
        NinjasLogger.log(name + "/leftY", inputs.leftY);
        NinjasLogger.log(name + "/rightX", inputs.rightX);
        NinjasLogger.log(name + "/rightY", inputs.rightY);
        NinjasLogger.log(name + "/L2Axis", inputs.L2Axis);
        NinjasLogger.log(name + "/R2Axis", inputs.R2Axis);
    }

    // Face buttons
    public Trigger cross() { return new Trigger(() -> inputs.cross); }
    public Trigger circle() { return new Trigger(() -> inputs.circle); }
    public Trigger square() { return new Trigger(() -> inputs.square); }
    public Trigger triangle() { return new Trigger(() -> inputs.triangle); }

    // D-Pad
    public Trigger povUp() { return new Trigger(() -> inputs.povUp); }
    public Trigger povDown() { return new Trigger(() -> inputs.povDown); }
    public Trigger povLeft() { return new Trigger(() -> inputs.povLeft); }
    public Trigger povRight() { return new Trigger(() -> inputs.povRight); }

    // Stick buttons
    public Trigger L3() { return new Trigger(() -> inputs.L3); }
    public Trigger R3() { return new Trigger(() -> inputs.R3); }

    // Bumpers
    public Trigger L1() { return new Trigger(() -> inputs.L1); }
    public Trigger R1() { return new Trigger(() -> inputs.R1); }

    // Triggers as buttons
    public Trigger L2() { return new Trigger(() -> inputs.L2); }
    public Trigger R2() { return new Trigger(() -> inputs.R2); }

    // Options / System
    public Trigger create() { return new Trigger(() -> inputs.create); }
    public Trigger options() { return new Trigger(() -> inputs.options); }
    public Trigger ps() { return new Trigger(() -> inputs.ps); }
    public Trigger touchpad() { return new Trigger(() -> inputs.touchpad); }

    // Axes
    public double getLeftX() { return inputs.leftX; }
    public double getLeftY() { return inputs.leftY; }
    public double getRightX() { return inputs.rightX; }
    public double getRightY() { return inputs.rightY; }
    public double getL2Axis() { return inputs.L2Axis; }
    public double getR2Axis() { return inputs.R2Axis; }
}
