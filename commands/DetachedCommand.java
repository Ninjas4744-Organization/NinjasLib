package frc.lib.NinjasLib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An {@link InstantCommand} that hands {@code command} off to the {@link CommandScheduler} as its
 * own independently-scheduled command, instead of running it inline. Use this when a command needs
 * to declare {@code requirements} (e.g. so it can be sequenced or requires-checked alongside other
 * commands) but the actual work should keep running on its own afterwards, detached from the
 * lifecycle of whatever scheduled this - for example, kicking off a long-running command from
 * inside a sequential group without the group waiting for it or cancelling it when the group ends.
 */
public class DetachedCommand extends InstantCommand {
    /**
     * @param command The command to schedule independently when this command runs.
     * @param requirements The subsystems this instant command itself requires (not necessarily the
     *     same as {@code command}'s requirements).
     */
    public DetachedCommand(Command command, Subsystem... requirements) {
        super(() -> CommandScheduler.getInstance().schedule(command), requirements);
    }
}
