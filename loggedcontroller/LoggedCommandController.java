package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.util.NinjasLogger;

/**
 * Wraps a {@link LoggedCommandControllerIO} (PS5 or Xbox gamepad) to provide AdvantageKit logging
 * of every button, POV, and axis on the controller, plus convenience accessors for binding
 * commands. Each button/POV is exposed as a {@link Trigger} suitable for {@code .onTrue(...)} /
 * {@code .whileTrue(...)} bindings, and each axis is exposed as a plain {@code double} getter.
 * Call {@link #periodic()} once per loop to refresh the underlying inputs and log them.
 */
public class LoggedCommandController {
    private LoggedCommandControllerIO.LoggedCommandControllerIOInputs inputs = new LoggedCommandControllerIO.LoggedCommandControllerIOInputs();
    private LoggedCommandControllerIO io;
    private String name;

    /**
     * Creates a logged controller wrapping the given IO layer.
     *
     * @param name the AdvantageKit log key prefix this controller's inputs are logged under
     * @param io   the hardware-specific IO implementation (e.g. {@link LoggedCommandControllerIOPS5}
     *             or {@link LoggedCommandControllerIOXbox}) to poll each cycle
     */
    public LoggedCommandController(String name, LoggedCommandControllerIO io) {
        this.io = io;
        this.name = name;
    }

    /**
     * Polls the controller hardware via {@link #io} and logs every button, POV, and axis value to
     * AdvantageKit under {@code name}. Must be called once per robot loop (e.g. from
     * {@code robotPeriodic()}) so that the {@link Trigger}s and axis getters returned by this
     * class reflect the current controller state.
     */
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
    /** @return a {@link Trigger} that is active while the Cross (PS5) / A (Xbox) button is held. */
    public Trigger cross() { return new Trigger(() -> inputs.cross); }
    /** @return a {@link Trigger} that is active while the Circle (PS5) / B (Xbox) button is held. */
    public Trigger circle() { return new Trigger(() -> inputs.circle); }
    /** @return a {@link Trigger} that is active while the Square (PS5) / X (Xbox) button is held. */
    public Trigger square() { return new Trigger(() -> inputs.square); }
    /** @return a {@link Trigger} that is active while the Triangle (PS5) / Y (Xbox) button is held. */
    public Trigger triangle() { return new Trigger(() -> inputs.triangle); }

    // D-Pad
    /** @return a {@link Trigger} that is active while the D-Pad (POV) is pressed up. */
    public Trigger povUp() { return new Trigger(() -> inputs.povUp); }
    /** @return a {@link Trigger} that is active while the D-Pad (POV) is pressed down. */
    public Trigger povDown() { return new Trigger(() -> inputs.povDown); }
    /** @return a {@link Trigger} that is active while the D-Pad (POV) is pressed left. */
    public Trigger povLeft() { return new Trigger(() -> inputs.povLeft); }
    /** @return a {@link Trigger} that is active while the D-Pad (POV) is pressed right. */
    public Trigger povRight() { return new Trigger(() -> inputs.povRight); }

    // Stick buttons
    /** @return a {@link Trigger} that is active while the left stick button is held. */
    public Trigger L3() { return new Trigger(() -> inputs.L3); }
    /** @return a {@link Trigger} that is active while the right stick button is held. */
    public Trigger R3() { return new Trigger(() -> inputs.R3); }

    // Bumpers
    /** @return a {@link Trigger} that is active while the left bumper is held. */
    public Trigger L1() { return new Trigger(() -> inputs.L1); }
    /** @return a {@link Trigger} that is active while the right bumper is held. */
    public Trigger R1() { return new Trigger(() -> inputs.R1); }

    // Triggers as buttons
    /** @return a {@link Trigger} that is active while the left trigger is held past its digital threshold. */
    public Trigger L2() { return new Trigger(() -> inputs.L2); }
    /** @return a {@link Trigger} that is active while the right trigger is held past its digital threshold. */
    public Trigger R2() { return new Trigger(() -> inputs.R2); }

    // Options / System
    /** @return a {@link Trigger} that is active while the Create (PS5) / Back (Xbox) button is held. */
    public Trigger create() { return new Trigger(() -> inputs.create); }
    /** @return a {@link Trigger} that is active while the Options (PS5) / Start (Xbox) button is held. */
    public Trigger options() { return new Trigger(() -> inputs.options); }
    /** @return a {@link Trigger} that is active while the PlayStation logo button is held. Never active on an Xbox controller. */
    public Trigger ps() { return new Trigger(() -> inputs.ps); }
    /** @return a {@link Trigger} that is active while the touchpad is pressed. Never active on an Xbox controller. */
    public Trigger touchpad() { return new Trigger(() -> inputs.touchpad); }

    // Axes
    /** @return the left stick X axis, in the range [-1, 1]. */
    public double getLeftX() { return inputs.leftX; }
    /** @return the left stick Y axis, in the range [-1, 1]. */
    public double getLeftY() { return inputs.leftY; }
    /** @return the right stick X axis, in the range [-1, 1]. */
    public double getRightX() { return inputs.rightX; }
    /** @return the right stick Y axis, in the range [-1, 1]. */
    public double getRightY() { return inputs.rightY; }
    /** @return the left trigger axis, in the range [0, 1]. */
    public double getL2Axis() { return inputs.L2Axis; }
    /** @return the right trigger axis, in the range [0, 1]. */
    public double getR2Axis() { return inputs.R2Axis; }
}
