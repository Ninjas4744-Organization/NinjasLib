package frc.lib.NinjasLib.subsystem;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;

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

    public static class BasicIOController implements All<ControllerIOInputsAutoLogged> {
        private Controller controller;
        private final Controller.ControllerType type;
        private final ControllerConstants constants;

        public BasicIOController(Controller.ControllerType type, ControllerConstants constants) {
            this.type = type;
            this.constants = constants;
        }

        @Override
        public void setup() {
            controller = Controller.createController(type, constants);
        }

        @Override
        public void updateInputs(ControllerIOInputsAutoLogged inputs) {
            controller.updateInputs(inputs);
        }

        @Override
        public void periodic() {
            controller.periodic();
        }

        @Override
        public void setPosition(double position) {
            controller.setPosition(position);
        }

        @Override
        public void setVelocity(double velocity) {
            controller.setVelocity(velocity);
        }

        @Override
        public void setPercent(double percent) {
            controller.setPercent(percent);
        }

        @Override
        public void stopMotor() {
            controller.stop();
        }

        @Override
        public void setEncoder(double position) {
            controller.setEncoder(position);
        }
    }
}