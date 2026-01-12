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
     * Your typical IO behavior.
     * Lets you control the inputs and outputs of the motor/s - such as setting the motor's percent voltage, the encoder's position, and the subsystem's physical position.
     */
    public interface Controllable<INPUTS> extends BaseIO<INPUTS> {
        default void setPosition(Object position) {}
        default void setVelocity(double velocity) {}
        default void setPercent(double percent) {}
        default void setEncoder(double position) {}
    }
}