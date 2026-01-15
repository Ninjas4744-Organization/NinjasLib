package frc.lib.NinjasLib.subsystem_interfaces;

public final class IO {
    private IO() {} // Prevent instantiation

    /**
     * Base interface that all SubsystemIO interfaces extend
     * <p>
     * Includes methods for the lifecycle of the SubsystemIO
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
    public interface PositionControlled<INPUTS> extends BaseIO<INPUTS> {
        default void setPosition(double position) {}
    }

    /**
     * Velocity controlled IO behavior.
     * Control the velocity of the motor.
     */
    public interface VelocityControlled<INPUTS> extends BaseIO<INPUTS> {
        default void setVelocity(double velocity) {}
    }

    /**
     * Percent controlled IO behavior.
     * Control the percent of the motor.
     */
    public interface PercentControlled<INPUTS> extends BaseIO<INPUTS> {
        default void setPercent(double percent) {}
    }

    /**
     * Access to encoder.
     */
    public interface Encoder<INPUTS> extends BaseIO<INPUTS> {
        default void setEncoder(double position) {}
    }
}