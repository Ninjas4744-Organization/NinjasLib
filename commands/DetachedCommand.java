package frc.lib.NinjasLib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DetachedCommand extends InstantCommand {
    public DetachedCommand(Command command, Subsystem... requirements) {
        super(command::schedule, requirements);
    }
}
