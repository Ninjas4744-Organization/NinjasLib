package frc.lib.NinjasLib.subsystem_interfaces;

import edu.wpi.first.wpilibj2.command.Command;

public final class ISubsystem {
    private ISubsystem() {} // Prevent instantiation

    /** Can be reset to a known baseline state. */
    public interface Resettable {
        boolean isReset();
        Command reset();
    }

    /** Requires logic to run periodically. */
    public interface Periodicable {
        void periodic();
    }

    /** Supports commanding and reading the physical position of the subsystem. */
    public interface PositionControlled<MEASUREMENT> {
        Command setPosition(MEASUREMENT position);
        MEASUREMENT getPosition();
    }

    /** Supports commanding and reading an angular position. */
    public interface AngleControlled<MEASUREMENT> {
        Command setAngle(MEASUREMENT angle);
        MEASUREMENT getAngle();
    }

    /** Supports commanding and reading a normalized percent output. */
    public interface PercentControlled {
        Command setPercent(double percent);
        double getPercent();
    }

    /**
     * Supports commanding the velocity of the subsystem. the measurement is for the user to decide.
     */
    public interface VelocityControlled {
        Command setVelocity(double velocity);
    }

    /** Can report whether its control goal has been reached. */
    public interface GoalOriented {
        boolean atGoal();
    }

    /** Can be commanded to immediately stop all motions or output. */
    public interface Stoppable {
        Command stop();
    }
}
