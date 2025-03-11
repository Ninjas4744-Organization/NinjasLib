package com.ninjas4744.NinjasLib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class StateMachineIO<StateEnum> extends StateMachineSubsystem<StateEnum> {
    private static StateMachineIO _instance;
//    private final Map<StateEnum, List<StateEndCondition<StateEnum>>> _endConditionMap;
    private final Map<StateEnum, List<Command>> _CommandMap;


    public static StateMachineIO getInstance() {
        if(_instance == null)
            throw new RuntimeException("StateMachineIO not initialized. Initialize StateMachineIO by setInstance() first.");
        return _instance;
    }

    public static void setInstance(StateMachineIO instance) {
        _instance = instance;
    }

    protected StateMachineIO(boolean paused) {
        super(paused);
        _CommandMap= new HashMap<>();
//        _endConditionMap = new HashMap<>();

        if(!paused)
//            setEndConditionMap();
            setCommandMap();


    }

    public void setTriggerForSimulationTesting(Trigger trigger) {
        trigger.onTrue(Commands.runOnce(
            () -> {
                if(!RobotStateIO.isSimulated())
                    return;

                if(_endConditionMap.get(RobotStateIO.getInstance().getRobotState()) != null)
                    changeRobotState(_endConditionMap.get(RobotStateIO.getInstance().getRobotState()).get(0).nextState);
            })
        );
    }

    /**
     * Sets the state of the robot to the given state only if possible. For example if the current
     * state is AMP_OUTAKE_READY it cannot change to PREPARE_AMP_OUTAKE
     *
     * @param wantedState - the state to change the robot state to
     */
    public void changeRobotState(StateEnum wantedState){
        if(canChangeRobotState((StateEnum) RobotStateIO.getInstance().getRobotState(), wantedState))
            RobotStateIO.getInstance().setRobotState(wantedState);
    }

    /**
     * Whether the robot can change from the current state to the wanted state, is it logical?
     * @param currentState the current state of the robot
     * @param wantedState the wanted state
     * @return true if robot can change
     */
    protected abstract boolean canChangeRobotState(StateEnum currentState, StateEnum wantedState);

    /**
     * Set in this function the end condition for each state with _endConditionMap
     */
    protected abstract void setEndConditionMap();
    protected abstract void setCommandMap();


    protected void addEndCondition(StateEnum state, StateEndCondition<StateEnum> endCondition) {
        if(!_endConditionMap.containsKey(state))
            _endConditionMap.put(state, new ArrayList<>(List.of(endCondition)));
        else
            _endConditionMap.get(state).add(endCondition);
    }
    protected void addCommand(StateEnum state, Command newCommand) {
        if(!_endConditionMap.containsKey(state))
            _endConditionMap.put(state, new ArrayList<>(List.of(endCondition)));
        else
            _endConditionMap.get(state).add(endCondition);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_endConditionMap.get(RobotStateIO.getInstance().getRobotState()) == null)
            return;

        for(StateEndCondition<StateEnum> endCondition : _endConditionMap.get(RobotStateIO.getInstance().getRobotState())){
            if (endCondition.condition.getAsBoolean())
                changeRobotState(endCondition.nextState);
        }
    }
}
