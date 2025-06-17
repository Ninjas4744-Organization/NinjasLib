package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.subsystems.example.ExampleSubsystemIOController;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private CommandPS5Controller driverController;
    private CommandPS5Controller operatorController;

    public RobotContainer() {
        RobotState.setInstance(new RobotState());

        switch (Constants.kCurrentMode) {
            case REAL, SIM:
                ExampleSubsystem.createInstance(new ExampleSubsystem(false, new ExampleSubsystemIOController()));
                break;

            case REPLAY:
                ExampleSubsystem.createInstance(new ExampleSubsystem(false, new ExampleSubsystemIO() {}));
                break;
        }

        driverController = new CommandPS5Controller(Constants.kDriverControllerPort);
        operatorController = new CommandPS5Controller(Constants.kOperatorControllerPort);

        configureBindings();
    }

    private void configureBindings() {

    }

    public void periodic() {
        Logger.recordOutput("Output1", 5);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
