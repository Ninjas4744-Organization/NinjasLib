package frc.lib.NinjasLib.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongConsumer;

/**
 * Runs a sequence of commands one after another, like WPILib's {@code SequentialCommandGroup}, but
 * with one key difference: once finished, {@link #isFinished()} keeps returning {@code true}
 * instead of the finished status being tied to the current command index. This makes it suitable
 * for use as a {@link frc.lib.NinjasLib.statemachine.StateMachineBase} state-end condition (see
 * {@code StateMachineBase.addStateEnd}), where the state machine polls {@code isFinished()} once
 * per loop and needs a stable answer rather than one that could flap. Commands passed in here
 * should only represent waiting for an event (e.g. {@code waitUntil}/{@code waitTime}), not actual
 * robot logic.
 */
public class StateEndCommand extends Command {
    private final List<Command> m_commands = new ArrayList();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private Command.InterruptionBehavior m_interruptBehavior;
    private boolean isFinished = false;

    /**
     * @param commands The commands to run in sequence, in order.
     */
    public StateEndCommand(Command... commands) {
        this.m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
        this.addCommands(commands);
    }

    /**
     * Appends commands to the end of the sequence, registering them with the
     * {@link CommandScheduler} as composed and merging their requirements, disabled-run behavior,
     * and interruption behavior into this command's own.
     *
     * @param commands The commands to add.
     * @throws IllegalStateException if this composition is currently running.
     */
    public final void addCommands(Command... commands) {
        if (this.m_currentCommandIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        } else {
            CommandScheduler.getInstance().registerComposedCommands(commands);

            for(Command command : commands) {
                this.m_commands.add(command);
                this.addRequirements(command.getRequirements());
                this.m_runWhenDisabled &= command.runsWhenDisabled();
                if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                    this.m_interruptBehavior = InterruptionBehavior.kCancelSelf;
                }
            }

        }
    }

    /**
     * Resets the sequence to its first command and initializes it. Called once when this command
     * is scheduled.
     */
    public final void initialize() {
        this.m_currentCommandIndex = 0;
        isFinished = false;
        if (!this.m_commands.isEmpty()) {
            ((Command)this.m_commands.get(0)).initialize();
        }
    }

    /**
     * Executes the currently active command in the sequence and, once it finishes, ends it and
     * initializes the next one.
     */
    public final void execute() {
        if (!this.m_commands.isEmpty()) {
            Command currentCommand = (Command)this.m_commands.get(this.m_currentCommandIndex);
            currentCommand.execute();
            if (currentCommand.isFinished()) {
                currentCommand.end(false);
                ++this.m_currentCommandIndex;
                if (this.m_currentCommandIndex < this.m_commands.size()) {
                    ((Command)this.m_commands.get(this.m_currentCommandIndex)).initialize();
                }
            }

        }
    }

    /**
     * If interrupted while a command in the sequence is still active, ends that command as
     * interrupted. Resets the internal index so the sequence can be run again.
     *
     * @param interrupted Whether this command was interrupted, as opposed to finishing normally.
     */
    public final void end(boolean interrupted) {
        if (interrupted && !this.m_commands.isEmpty() && this.m_currentCommandIndex > -1 && this.m_currentCommandIndex < this.m_commands.size()) {
            ((Command)this.m_commands.get(this.m_currentCommandIndex)).end(true);
        }

        this.m_currentCommandIndex = -1;
    }

    /**
     * @return {@code true} once every command in the sequence has run to completion. Unlike
     *     {@code SequentialCommandGroup}, this latches: once {@code true}, it keeps returning
     *     {@code true} even if called again before {@link #end(boolean)} resets the sequence - see
     *     the class Javadoc for why this matters.
     */
    public final boolean isFinished() { // This is the difference from SequentialCommandGroup. Once finished, the command stays finished.
        if (!isFinished)
            isFinished = this.m_currentCommandIndex == this.m_commands.size();
        return isFinished;
    }

    /**
     * @return Whether this command should run while the robot is disabled - {@code true} only if
     *     every command in the sequence also runs when disabled.
     */
    public boolean runsWhenDisabled() {
        return this.m_runWhenDisabled;
    }

    /**
     * @return The interruption behavior for this composition - {@code kCancelSelf} if any
     *     composed command requests it, otherwise {@code kCancelIncoming}.
     */
    public Command.InterruptionBehavior getInterruptionBehavior() {
        return this.m_interruptBehavior;
    }

    /**
     * Publishes the index of the currently running command under the {@code "index"} sendable
     * property, in addition to the base {@link Command} sendable data.
     *
     * @param builder The sendable builder to publish to.
     */
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addIntegerProperty("index", () -> (long)this.m_currentCommandIndex, (LongConsumer)null);
    }
}
