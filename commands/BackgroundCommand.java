package frc.lib.NinjasLib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public class BackgroundCommand {
    private Command command;

    public boolean isRunning() {
        return command != null && command.isScheduled() && !command.isFinished();
    }

    public void stop() {
        if (isRunning())
            command.cancel();
    }

    public void setNewTask(Command task) {
        stop();

        command = task;
        CommandScheduler.getInstance().schedule(command);
    }

    public Command setNewTaskCommand(Command task) {
        return Commands.runOnce(() -> setNewTask(task));
    }

    public Command setNewTaskDynamic(Supplier<Command> task) {
        return Commands.runOnce(() -> setNewTask(task.get()));
    }

    public Command getTask() {
        return command;
    }
}
