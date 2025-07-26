package frc.lib.NinjasLib.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.IntSupplier;

public class LoopCommand extends Command {
    private Command command;
    private int repeat;
    private boolean commandEnded;
    private boolean ended;
    private IntSupplier n;

    public LoopCommand(Command command, IntSupplier n) {
        this.command = command;
        this.n = n;
    }

    @Override
    public final void initialize() {
        command.initialize();
        repeat = 0;
        commandEnded = false;
    }

    @Override
    public final void execute() {
        if (ended)
            return;

        if (commandEnded) {
            commandEnded = false;
            repeat++;

            if (repeat == n.getAsInt())
                return;

            command.initialize();
        }
        command.execute();
        if (command.isFinished()) {
            // restart command
            command.end(false);
            commandEnded = true;
        }
    }

    @Override
    public final void end(boolean interrupted) {
        // Make sure we didn't already call end() (which would happen if the command finished in the
        // last call to our execute())
        if (!ended) {
            command.end(interrupted);
            commandEnded = true;
            ended = true;
        }
    }

    @Override
    public final boolean isFinished() {
        return repeat == n.getAsInt() || ended;
    }

    @Override
    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return command.getInterruptionBehavior();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("repeat", () -> repeat, null);
    }
}
