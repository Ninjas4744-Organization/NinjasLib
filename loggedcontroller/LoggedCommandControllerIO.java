package frc.lib.NinjasLib.loggedcontroller;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

/**
 * Hardware-agnostic IO interface for a game controller (PS5 or Xbox gamepad). Implementations
 * poll the underlying controller hardware once per loop and return the raw button, POV, and axis
 * states as a {@link LoggedCommandControllerIOInputs}, which {@link LoggedCommandController} then
 * logs via AdvantageKit and exposes as {@link edu.wpi.first.wpilibj2.command.button.Trigger}s.
 * See {@link LoggedCommandControllerIOPS5} and {@link LoggedCommandControllerIOXbox} for the
 * concrete, controller-specific implementations.
 */
public interface LoggedCommandControllerIO {
    /**
     * A snapshot of every button, POV, and axis state read from a controller in a single
     * {@link #update()} call. Both PlayStation-style fields (cross/circle/square/triangle,
     * create/options, ps, touchpad) and Xbox-style fields (A/B/X/Y, back/start) are always
     * populated with the same underlying state, regardless of which controller is connected, so
     * callers can bind to whichever naming convention fits the driver station layout in use.
     */
    class LoggedCommandControllerIOInputs {
        // Face buttons
        /** Cross (PS5) face button; mirrors {@link #A}. */
        public boolean cross;
        /** Circle (PS5) face button; mirrors {@link #B}. */
        public boolean circle;
        /** Square (PS5) face button; mirrors {@link #X}. */
        public boolean square;
        /** Triangle (PS5) face button; mirrors {@link #Y}. */
        public boolean triangle;

        // Face buttons XBOX
        /** Xbox "A" face button; mirrors {@link #cross}. */
        public boolean A;
        /** Xbox "B" face button; mirrors {@link #circle}. */
        public boolean B;
        /** Xbox "X" face button; mirrors {@link #square}. */
        public boolean X;
        /** Xbox "Y" face button; mirrors {@link #triangle}. */
        public boolean Y;

        // D-Pad (POV)
        /** {@code true} when the D-Pad (POV) is pressed up (POV angle 0&deg;). */
        public boolean povUp;
        /** {@code true} when the D-Pad (POV) is pressed down (POV angle 180&deg;). */
        public boolean povDown;
        /** {@code true} when the D-Pad (POV) is pressed left (POV angle 270&deg;). */
        public boolean povLeft;
        /** {@code true} when the D-Pad (POV) is pressed right (POV angle 90&deg;). */
        public boolean povRight;

        // Stick buttons
        /** Left stick button (pressing down on the stick). */
        public boolean L3;
        /** Right stick button (pressing down on the stick). */
        public boolean R3;

        // Bumpers
        /** Left bumper. */
        public boolean L1;
        /** Right bumper. */
        public boolean R1;

        // Triggers (as digital buttons)
        /** Left trigger, read as a digital button (pressed past a threshold). */
        public boolean L2;
        /** Right trigger, read as a digital button (pressed past a threshold). */
        public boolean R2;

        // Options / System
        /** PS5 "Create" button, left of the touchpad; mirrors {@link #back}. */
        public boolean create;    // left of touchpad
        /** PS5 "Options" button, right of the touchpad; mirrors {@link #start}. */
        public boolean options;   // right of touchpad
        /** PlayStation logo button. Always {@code false} on an Xbox controller. */
        public boolean ps;        // PlayStation logo button
        /** Touchpad press. Always {@code false} on an Xbox controller. */
        public boolean touchpad;  // touchpad press

        // Options / System XBOX
        /** Xbox "Back" button; mirrors {@link #create}. */
        public boolean back;    // left of touchpad
        /** Xbox "Start" button; mirrors {@link #options}. */
        public boolean start;   // right of touchpad

        // Axes (joysticks and triggers)
        /** Left stick X axis, in the range [-1, 1]. */
        public double leftX;
        /** Left stick Y axis, in the range [-1, 1]. */
        public double leftY;
        /** Right stick X axis, in the range [-1, 1]. */
        public double rightX;
        /** Right stick Y axis, in the range [-1, 1]. */
        public double rightY;
        /** Left trigger axis, in the range [0, 1]. */
        public double L2Axis;
        /** Right trigger axis, in the range [0, 1]. */
        public double R2Axis;
    }

    /**
     * Polls the controller hardware and returns a fresh snapshot of all button, POV, and axis
     * states. Called once per loop by {@link LoggedCommandController#periodic()}.
     *
     * @return the latest {@link LoggedCommandControllerIOInputs}
     */
    LoggedCommandControllerIOInputs update();
}
