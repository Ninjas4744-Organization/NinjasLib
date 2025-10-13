package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jgrapht.Graph;
import org.jgrapht.graph.SimpleDirectedGraph;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public abstract class StateMachineBase<StateEnum extends Enum<StateEnum>> extends SubsystemBase {
    private static StateMachineBase instance;
    protected Graph<StateEnum, Command> graph;
    protected Map<StateEnum, Map<BooleanSupplier, StateEnum>> stateEnds;
    protected Command currentEdge;

    public static StateMachineBase getInstance() {
        if (instance == null)
            throw new RuntimeException("StateMachineBase not initialized. Initialize StateMachineBase by setInstance() first.");
        return instance;
    }

    public static void setInstance(StateMachineBase instance) {
        StateMachineBase.instance = instance;
        StateMachineBase.instance.defineGraph();
    }

    public StateMachineBase(Class<StateEnum> states) {
        graph = new SimpleDirectedGraph<>(Command.class);
        stateEnds = new HashMap<>();

        for(StateEnum state : states.getEnumConstants()) {
            graph.addVertex(state);
        }
    }

    @Override
    public void periodic() {
        // Check state ends for current state
        if(!isTransitioning()) {
            Map<BooleanSupplier, StateEnum> ends = stateEnds.get((StateEnum) RobotStateBase.getInstance().getRobotState());
            if(ends != null) {
                for(BooleanSupplier end : ends.keySet()) {
                    if(end.getAsBoolean()) {
                        changeRobotState(ends.get(end), false);
                        break;
                    }
                }
            }
        }

        // Check if transition command ended
        if(currentEdge != null && currentEdge.isFinished()) {
            RobotStateBase.getInstance().setRobotState(graph.getEdgeTarget(currentEdge));
            currentEdge = null;
        }
    }

    /**
     * Tries to set the state of the robot to the given state by the connecting edge between the current state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     * @param force Whether to transition to a robot state even though the robot is currently already transitioning to a state.
     */
    public void changeRobotState(StateEnum wantedState, boolean force) {
        if(isTransitioning() && !force)
            return;

        StateEnum currentState = (StateEnum) RobotStateBase.getInstance().getRobotState();

        Command edge = graph.getEdge(currentState, wantedState);
        if(edge != null) {
            if (currentEdge != null)
                currentEdge.cancel();

            currentEdge = edge;
            currentEdge.schedule();
        }
    }

    public boolean isTransitioning() {
        return currentEdge != null;
    }

    /**
     * Set in this function the commands to run in state transitions.
     * And the state ends suppliers.
     *
     * @see #addEdge(Enum, Enum, Command)
     * @see #addStateEnd(Enum, Map)
     */
    protected abstract void defineGraph();

    protected void addEdge(StateEnum start, StateEnum end, Command command) {
        graph.addEdge(start, end, command);
    }

    protected void addEdge(StateEnum start, StateEnum end) {
        addEdge(start, end, Commands.none());
    }

    protected void addMultiEdge(StateEnum end, Command command, StateEnum... start) {
        for(StateEnum state : start){
            addEdge(state, end, command);
        }
    }

    protected void addMultiEdge(StateEnum end, StateEnum... start) {
        addMultiEdge(end, Commands.none(), start);
    }

    protected void addOmniEdge(Class<StateEnum> states, StateEnum end, Command command) {
        for(StateEnum state : states.getEnumConstants()) {
            addEdge(state, end, command);
        }
    }

    protected void addCommutativeEdge(StateEnum start, StateEnum end, Command command) {
        addEdge(start, end, command);
        addEdge(end, start, command);
    }

    protected void addCommutativeEdge(StateEnum start, StateEnum end) {
        addEdge(start, end);
        addEdge(end, start);
    }

    protected void addCommutativeEdge(StateEnum start, StateEnum end, Command forward, Command backward) {
        addEdge(start, end, forward);
        addEdge(end, start, backward);
    }

    protected void addStateEnd(StateEnum state, Map<BooleanSupplier, StateEnum> nextStatesMap) {
        stateEnds.put(state, nextStatesMap);
    }
}
