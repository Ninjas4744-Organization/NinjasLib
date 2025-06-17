package frc.robot.subsystems.example;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ExampleSubsystemIO {
    @AutoLog
    class ExampleSubsystemIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void setAngle(Rotation2d angle) {
    }

    default void setPercent(double percent) {
    }

    default void updateInputs(ExampleSubsystemIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
