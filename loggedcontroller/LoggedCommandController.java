package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class LoggedCommandController {
    private LoggedCommandControllerIOInputsAutoLogged inputs = new LoggedCommandControllerIOInputsAutoLogged();
    private LoggedCommandControllerIO io;
    private String name;

    public LoggedCommandController(String name, LoggedCommandControllerIO io) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name + " Controller", inputs);
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
