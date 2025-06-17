package frc.robot.subsystems.example;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class ExampleSubsystemIOController implements ExampleSubsystemIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kExampleSubsystemControllerConstants);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        controller.setPosition(angle.getRadians());
    }

    @Override
    public void setPercent(double percent) {
        controller.setPercent(percent);
    }

    @Override
    public void updateInputs(ExampleSubsystemIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }
}