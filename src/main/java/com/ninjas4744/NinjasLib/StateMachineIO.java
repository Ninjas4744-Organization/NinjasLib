package com.ninjas4744.NinjasLib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;

import java.util.HashMap;
import java.util.Map;

public abstract class StateMachineIO<StateEnum> extends StateMachineSubsystem<StateEnum> {
    private static StateMachineIO _instance;
    protected final Map<StateEnum, Command> _commandMap;
    protected Command _currentCommand;

    public static StateMachineIO getInstance() {
        if(_instance == null)
            throw new RuntimeException("StateMachineIO not initialized. Initialize StateMachineIO by setInstance() first.");
        return _instance;
    }

    public static void setInstance(StateMachineIO instance) {
        _instance = instance;

        if(!_instance._paused)
            _instance.setCommandMap();
    }

    protected StateMachineIO(boolean paused) {
        super(paused);
        _commandMap = new HashMap<>();
    }

    public void setTriggerForSimulationTesting(Trigger trigger) {
        //Work In Progress
//        trigger.onTrue(Commands.runOnce(
//            () -> {
//                if(RobotStateIO.isSimulated())
//                    _currentCommand.
//            })
//        );
    }

    /**
     * Sets the state of the robot to the given state only if possible
     *
     * @param wantedState - the state to change the robot state to
     * @see #canChangeRobotState(StateEnum, StateEnum)
     */
    public void changeRobotState(StateEnum wantedState){
        if(canChangeRobotState((StateEnum) RobotStateIO.getInstance().getRobotState(), wantedState)){
            RobotStateIO.getInstance().setRobotState(wantedState);

            if(_commandMap.get(wantedState) != null){
                if(_currentCommand != null)
                    _currentCommand.cancel();

                _currentCommand = _commandMap.get(wantedState);
                _currentCommand.schedule();
            }
        }
    }

    /**
     * Whether the robot can change from the current state to the wanted state, is it logical?
     * @param currentState the current state of the robot
     * @param wantedState the wanted state
     * @return true if robot can change
     */
    protected abstract boolean canChangeRobotState(StateEnum currentState, StateEnum wantedState);

    /**
     * Set in this function the command to run in each state
     * @see #addCommand(StateEnum, Command)
     */
    protected abstract void setCommandMap();

    /**
     * adds a command to the commands hashmap. The command will run when robot switches to the given state
     * @param state the state in which to run the command
     * @param command the command to run when switching to the state
     */
    protected void addCommand(StateEnum state, Command command) {
        _commandMap.put(state, command);
    }
}
