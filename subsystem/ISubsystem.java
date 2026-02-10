package frc.lib.NinjasLib.subsystem;

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
        double getPosition();
        void setPosition(double position);
        Command setPositionCmd(double position);
    }

    /** Supports commanding and reading an angular position. */
    public interface AngleControlled {
        Rotation2d getAngle();
        void setAngle(Rotation2d angle);
        Command setAngleCmd(Rotation2d angle);
    }

    /** Supports commanding and reading the velocity. */
    public interface VelocityControlled {
        double getVelocity();
        void setVelocity(double velocity);
        Command setVelocityCmd(double velocity);
    }

    /** Supports commanding and reading the percent output. */
    public interface PercentControlled {
        double getOutput();
        void setPercent(double percent);
        Command setPercentCmd(double percent);
    }

    /** Supports reporting its control goal. */
    public interface GoalOriented<T> {
        boolean atGoal();
        T getGoal();
    }

    /** Supports being commanded to immediately stop all motions or output. */
    public interface Stoppable {
        void stop();
        Command stopCmd();
    }
}
