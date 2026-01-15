package frc.lib.NinjasLib.subsystem_interfaces;

import edu.wpi.first.wpilibj2.command.Command;

public final class ISubsystem {
    private ISubsystem() {} // Prevent instantiation

    /** Can be reset. */
    public interface Resettable {
        boolean isReset();
        Command reset();
    }

    /** Supports commanding and reading the position. */
    public interface PositionControlled<MEASUREMENT> {
        Command setPosition(MEASUREMENT position);
        MEASUREMENT getPosition();
    }

    /** Supports commanding and reading an angular position. */
    public interface AngleControlled<MEASUREMENT> {
        Command setAngle(MEASUREMENT angle);
        MEASUREMENT getAngle();
    }

    /** Supports commanding and reading the percent output. */
    public interface PercentControlled {
        Command setPercent(double percent);
        double getOutput();
    }

    /** Supports commanding and reading the velocity. */
    public interface VelocityControlled<MEASUREMENT> {
        Command setVelocity(MEASUREMENT velocity);
        MEASUREMENT getVelocity();
    }

    /** Supports reporting its control goal. */
    public interface GoalOriented {
        boolean atGoal();
        boolean getGoal();
    }

    /** Supports being commanded to immediately stop all motions or output. */
    public interface Stoppable {
        Command stop();
    }
}
