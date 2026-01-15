package frc.lib.NinjasLib.subsystem_interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class ISubsystem {
    private ISubsystem() {} // Prevent instantiation

    /** Can be reset. */
    public interface Resettable {
        boolean isReset();
        Command reset();
    }

    /** Supports commanding and reading the position. */
    public interface PositionControlled {
        Command setPosition(double position);
        double getPosition();
    }

    /** Supports commanding and reading an angular position. */
    public interface AngleControlled {
        Command setAngle(Rotation2d angle);
        Rotation2d getAngle();
    }

    /** Supports commanding and reading the velocity. */
    public interface VelocityControlled {
        Command setVelocity(double velocity);
        double getVelocity();
    }

    /** Supports commanding and reading the percent output. */
    public interface PercentControlled {
        Command setPercent(double percent);
        double getOutput();
    }

    /** Supports reporting its control goal. */
    public interface GoalOriented<T> {
        boolean atGoal();
        T getGoal();
    }

    /** Supports being commanded to immediately stop all motions or output. */
    public interface Stoppable {
        Command stop();
    }
}
