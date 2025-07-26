package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;

public abstract class StateMachineBase<StateEnum> {
    private static StateMachineBase instance;
    protected final Map<StateEnum, Command> commandMap;
    protected Command currentCommand;

    public static StateMachineBase getInstance() {
        if (instance == null)
            throw new RuntimeException("StateMachineIO not initialized. Initialize StateMachineIO by setInstance() first.");
        return instance;
    }

    public static void setInstance(StateMachineBase instance) {
        StateMachineBase.instance = instance;
        StateMachineBase.instance.setCommandMap();
    }

    public StateMachineBase() {
        commandMap = new HashMap<>();
    }

    /**
     * Sets the state of the robot to the given state only if possible
     *
     * @param wantedState The state to change the robot state to
     * @see #canChangeRobotState(StateEnum, StateEnum)
     */
    public void changeRobotState(StateEnum wantedState){
        if (canChangeRobotState((StateEnum) RobotStateBase.getInstance().getRobotState(), wantedState)) {
            RobotStateBase.getInstance().setRobotState(wantedState);

            if (commandMap.get(wantedState) != null) {
                if (currentCommand != null)
                    currentCommand.cancel();

                currentCommand = commandMap.get(wantedState);
                currentCommand.schedule();
            }
        }
    }

    /**
     * Whether the robot can change from the current state to the wanted state, is it logical?
     * @param currentState The current state of the robot
     * @param wantedState The wanted state
     * @return True if robot can change
     */
    protected abstract boolean canChangeRobotState(StateEnum currentState, StateEnum wantedState);

    /**
     * Set in this function the command to run in each state
     * @see #addCommand(StateEnum, Command)
     */
    protected abstract void setCommandMap();

    /**
     * Adds a command to the commands hashmap. The command will run when robot switches to the given state
     * @param state The state in which to run the command
     * @param command The command to run when switching to the state
     */
    protected void addCommand(StateEnum state, Command command) {
        commandMap.put(state, command);
    }
}
