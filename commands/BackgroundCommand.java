package frc.lib.NinjasLib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

/**
 * Holds a single "background task" slot: at most one {@link Command} runs through this object at
 * a time, and assigning a new task automatically cancels whatever was previously running in that
 * slot. This is not itself a {@link Command} - it's a small piece of state meant to be driven from
 * elsewhere (e.g. {@code periodic()} or a state machine) to manage a task whose lifetime spans
 * multiple state changes, such as {@link frc.lib.NinjasLib.statemachine.StateMachineBase}'s
 * per-state command.
 */
public class BackgroundCommand {
    private Command command;

    /**
     * @return Whether the current task is scheduled and has not yet finished.
     */
    public boolean isRunning() {
        return command != null && command.isScheduled() && !command.isFinished();
    }

    /**
     * Cancels the current task if it is running. Leaves the last-assigned task in place so
     * {@link #getTask()} still returns it.
     */
    public void stop() {
        if (isRunning())
            command.cancel();
    }

    /**
     * Stops the currently running task (if any) and schedules {@code task} as the new background
     * task in its place.
     *
     * @param task The command to run.
     */
    public void setNewTask(Command task) {
        stop();

        command = task;
        CommandScheduler.getInstance().schedule(command);
    }

    /**
     * @param task The command to run.
     * @return An instant command that runs {@link #setNewTask(Command)} with {@code task}.
     */
    public Command setNewTaskCommand(Command task) {
        return Commands.runOnce(() -> setNewTask(task));
    }

    /**
     * Same as {@link #setNewTaskCommand(Command)}, but the command to run is resolved lazily from
     * {@code task} at the moment this is scheduled, rather than fixed in advance.
     *
     * @param task Supplier producing the command to run.
     * @return An instant command that runs {@link #setNewTask(Command)} with the supplied task.
     */
    public Command setNewTaskDynamic(Supplier<Command> task) {
        return Commands.runOnce(() -> setNewTask(task.get()));
    }

    /**
     * @return The most recently assigned task, or {@code null} if none has been set. Note this is
     *     returned even after the task has finished or been stopped.
     */
    public Command getTask() {
        return command;
    }
}
