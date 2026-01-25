package frc.lib.NinjasLib.subsystem_interfaces;

public final class IO {
    private IO() {} // Prevent instantiation

    /**
     * Base interface that all Subsystems' IO's need.
     */
    public interface BaseIO<INPUTS> {
        default void setup() {}
        default void periodic() {}
        default void updateInputs(INPUTS inputs) {}
    }

    /**
     * Position controlled IO behavior.
     * Control the position of the motor.
     */
    public interface PositionControlled {
        default void setPosition(double position) {}
    }

    /**
     * Velocity controlled IO behavior.
     * Control the velocity of the motor.
     */
    public interface VelocityControlled {
        default void setVelocity(double velocity) {}
    }

    /**
     * Percent controlled IO behavior.
     * Control the percent of the motor.
     */
    public interface PercentControlled {
        default void setPercent(double percent) {}
    }

    /**
     * Access to encoder.
     */
    public interface Encoder {
        default void setEncoder(double position) {}
    }

    /**
     * Stoppable
     */
    public interface Stoppable {
        default void stopMotor() {}
    }

    public interface All<INPUTS> extends BaseIO<INPUTS>, PositionControlled, VelocityControlled, PercentControlled, Encoder, Stoppable {}
}