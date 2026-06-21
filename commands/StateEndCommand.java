package frc.lib.NinjasLib.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongConsumer;

public class StateEndCommand extends Command {
    private final List<Command> m_commands = new ArrayList();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private Command.InterruptionBehavior m_interruptBehavior;
    private boolean isFinished = false;

    public StateEndCommand(Command... commands) {
        this.m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
        this.addCommands(commands);
    }

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

    public final void initialize() {
        this.m_currentCommandIndex = 0;
        isFinished = false;
        if (!this.m_commands.isEmpty()) {
            ((Command)this.m_commands.get(0)).initialize();
        }
    }

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

    public final void end(boolean interrupted) {
        if (interrupted && !this.m_commands.isEmpty() && this.m_currentCommandIndex > -1 && this.m_currentCommandIndex < this.m_commands.size()) {
            ((Command)this.m_commands.get(this.m_currentCommandIndex)).end(true);
        }

        this.m_currentCommandIndex = -1;
    }

    public final boolean isFinished() { // This is the difference from SequentialCommandGroup. Once finished, the command stays finished.
        if (!isFinished)
            isFinished = this.m_currentCommandIndex == this.m_commands.size();
        return isFinished;
    }

    public boolean runsWhenDisabled() {
        return this.m_runWhenDisabled;
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return this.m_interruptBehavior;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addIntegerProperty("index", () -> (long)this.m_currentCommandIndex, (LongConsumer)null);
    }
}
