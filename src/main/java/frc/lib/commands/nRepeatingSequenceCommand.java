package frc.lib.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.IntSupplier;

public class nRepeatingSequenceCommand extends Command {
    private SequentialCommandGroup sequence;
    private int repeat;
    private boolean ended;
    private IntSupplier n;

    public nRepeatingSequenceCommand(IntSupplier n, Command... commands) {
        sequence = new SequentialCommandGroup(commands);
        this.n = n;
    }

    @Override
    public final void initialize() {
        sequence.initialize();
        repeat = 0;
        ended = false;
    }

    @Override
    public final void execute() {
        if (ended) {
            ended = false;
            repeat++;

            if (repeat == n.getAsInt())
                return;

            sequence.initialize();
        }
        sequence.execute();
        if (sequence.isFinished()) {
            // restart command
            sequence.end(false);
            ended = true;
        }
    }

    @Override
    public final void end(boolean interrupted) {
        // Make sure we didn't already call end() (which would happen if the command finished in the
        // last call to our execute())
        if (!ended) {
            sequence.end(interrupted);
            ended = true;
        }
    }

    @Override
    public final boolean isFinished() {
        return repeat == n.getAsInt();
    }

    @Override
    public boolean runsWhenDisabled() {
        return sequence.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return sequence.getInterruptionBehavior();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("repeat", () -> repeat, null);
    }
}
