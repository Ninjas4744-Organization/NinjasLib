package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ExampleSubsystem extends SubsystemBase {
    private ExampleSubsystemIO io;
    private final ExampleSubsystemIOInputsAutoLogged inputs = new ExampleSubsystemIOInputsAutoLogged();
    private static ExampleSubsystem instance;
    private boolean enabled;

    public static ExampleSubsystem getInstance() {
        return instance;
    }

    public static void createInstance(ExampleSubsystem exampleSubsystem) {
        instance = exampleSubsystem;
    }

    public ExampleSubsystem(boolean enabled, ExampleSubsystemIO io) {
        if (enabled) {
            this.io = io;
            io.setup();
        }
        this.enabled = enabled;
    }

    public ExampleSubsystemIO getIO() {
        if (enabled)
            return io;
        else
            return new ExampleSubsystemIO() {};
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("ExampleSubsystem", inputs);
    }
}