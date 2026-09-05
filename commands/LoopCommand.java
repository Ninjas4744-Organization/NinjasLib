package frc.lib.NinjasLib.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Wraps a command and runs it repeatedly, restarting it (calling {@code end()} then
 * {@code initialize()} again) each time it finishes, for a fixed number of repeats. Unlike
 * composing this by hand with {@code Commands.repeatingSequence}-style helpers, this class shares a
 * single wrapped {@link Command} instance across every repeat and tracks the repeat count as
 * sendable telemetry.
 */
public class LoopCommand extends Command {
    private Command command;
    private int repeat;
    private boolean commandEnded;
    private boolean ended;
    private int n;

    /**
     * @param command The command to run repeatedly.
     * @param n The number of times to run {@code command} before this command finishes.
     */
    public LoopCommand(Command command, int n) {
        this.command = command;
        this.n = n;
    }

    /**
     * Initializes the wrapped command and resets the repeat count. Called once when this command
     * is scheduled.
     */
    @Override
    public final void initialize() {
        command.initialize();
        repeat = 0;
        commandEnded = false;
        ended = false;
    }

    /**
     * Runs one cycle of the wrapped command. When the wrapped command reports finished, it is
     * ended and, unless the configured repeat count ({@code n}) has been reached, immediately
     * re-initialized so the next call resumes a fresh run.
     */
    @Override
    public final void execute() {
        if (ended)
            return;

        if (commandEnded) {
            commandEnded = false;
            repeat++;

            if (repeat >= n)
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

    /**
     * Ends the wrapped command, guarded so it is not ended twice if this command already finished
     * (and thus already ended the wrapped command) on the previous {@link #execute()} call.
     *
     * @param interrupted Whether this command was interrupted, as opposed to finishing normally.
     */
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

    /**
     * @return {@code true} once the wrapped command has completed {@code n} repeats (or this
     *     command has otherwise already ended).
     */
    @Override
    public final boolean isFinished() {
        return repeat == n || ended;
    }

    /**
     * @return Whether this command should run while the robot is disabled - delegates to the
     *     wrapped command.
     */
    @Override
    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }

    /**
     * @return The interruption behavior of the wrapped command, which this command adopts as its
     *     own.
     */
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return command.getInterruptionBehavior();
    }

    /**
     * Publishes the current repeat count under the {@code "repeat"} sendable property, in addition
     * to the base {@link Command} sendable data.
     *
     * @param builder The sendable builder to publish to.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("repeat", () -> repeat, null);
    }
}
