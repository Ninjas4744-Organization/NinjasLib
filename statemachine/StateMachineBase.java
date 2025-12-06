package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jgrapht.Graph;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public abstract class StateMachineBase<StateEnum extends Enum<StateEnum>> extends SubsystemBase {
    private static StateMachineBase instance;
    protected Graph<StateEnum, Command> graph;
    protected Map<StateEnum, Map<Command, StateEnum>> stateEnds;
    protected Command currentEdge;
    private Class<StateEnum> stateEnumClass;

    public static StateMachineBase getInstance() {
        if (instance == null)
            throw new RuntimeException("StateMachineBase not initialized. Initialize StateMachineBase by setInstance() first.");
        return instance;
    }

    public static void setInstance(StateMachineBase instance) {
        StateMachineBase.instance = instance;
        instance.defineGraph();
        instance.printGraph();
    }

    public StateMachineBase(Class<StateEnum> states) {
        graph = new SimpleDirectedGraph<>(Command.class);
        stateEnds = new HashMap<>();

        stateEnumClass = states;
        for(StateEnum state : states.getEnumConstants()) {
            graph.addVertex(state);
        }
    }

    @Override
    public void periodic() {
        // Check state ends for current state
        if(!isTransitioning()) {
            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    if((end.isFinished() || !end.isScheduled()) && canTransitionTo(ends.get(end))) {
                        System.out.println("[StateMachine] Ended state " + getCurrentState().name() + ": " + ends.get(end).name());
                        changeRobotState(ends.get(end), false);
                        break;
                    }
                }
            }
        }

        // Check if transition command ended
        if(currentEdge != null && (currentEdge.isFinished() || !currentEdge.isScheduled())) {
            System.out.println("[StateMachine] Ended " + getCurrentState().name() + " -> " + getTargetState().name());

            RobotStateBase.getInstance().setRobotState(getTargetState());
            currentEdge = null;

            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    end.schedule();
                }
            }
        }

        Logger.recordOutput("StateMachine/Is Transitioning", isTransitioning());
        Logger.recordOutput("StateMachine/Current State", getCurrentState());
        Logger.recordOutput("StateMachine/Target State", getTargetState() == null ? "N/A" : getTargetState().name());
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

        Command edge = graph.getEdge(getCurrentState(), wantedState);
        if(edge != null) {
            if (currentEdge != null)
                currentEdge.cancel();

            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    end.cancel();
                }
            }

            currentEdge = edge;
            currentEdge.schedule();

            System.out.println("[StateMachine] Started " + getCurrentState().name() + " -> " + getTargetState().name());
        }
    }

    /**
     * Tries to set the state of the robot to the given state by the connecting edge between the current state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     */
    public void changeRobotState(StateEnum wantedState) {
        changeRobotState(wantedState, false);
    }

    /**
     * @param start The start state
     * @param end The end state
     * @return Whether there is an edge command connecting the start state to the end state.
     */
    public boolean canTransitionTo(StateEnum start, StateEnum end) {
        return graph.containsEdge(start, end);
    }

    /**
     * @param state The state
     * @return Whether the robot can transition to this state: Whether there is an edge command connecting the current robot state to this state.
     */
    public boolean canTransitionTo(StateEnum state) {
        return graph.containsEdge(getCurrentState(), state);
    }

    /**
     * @return Whether the robot is currently transitioning from one state to another. Whether the statemachine is running an edge command.
     */
    public boolean isTransitioning() {
        return currentEdge != null;
    }

    /**
     * @return The current edge/transition command.
     * If the robot is transitioning from one state to another, the edge command will be returned. Otherwise, will return null.
     */
    public Command getCurrentTransitionCommand() {
        return currentEdge;
    }

    /**
     * @return The current robot state from RobotStateBase.
     */
    public StateEnum getCurrentState() {
        return (StateEnum) RobotStateBase.getInstance().getRobotState();
    }

    /**
     * @return The target state the statemachine is currently transitioning to.
     * If an edge command is running, the state the edge is transitioning to will be returned. Otherwise, will return null.
     */
    public StateEnum getTargetState() {
        if(currentEdge != null)
            return graph.getEdgeTarget(currentEdge);
        return null;
    }

    /**
     * Set in this function the commands to run in state transitions.
     * And the state end conditions.
     *
     * @see #addEdge(Enum, Enum, Command)
     * @see #addStateEnd(Enum, Map)
     */
    protected abstract void defineGraph();

    /**
     * Add an edge command connecting the start state to the end state in the statemachine's graph.
     * @param start The state the command will from.
     * @param end The state the robot will be when the command finishes.
     * @param command The transition command (Doesn't need to change the robot state, this is handled by the statemachine).
     */
    protected void addEdge(StateEnum start, StateEnum end, Command command) {
        if (start.equals(end)) {
            System.out.println("[StateMachine] Start and end state of an edge cannot be the same");
            return;
        }

        graph.addEdge(start, end, command);
    }

    /**
     * Add a connection from the start state to the end state in the statemachine's graph.
     * @param start The state to transition from.
     * @param end The state the robot will be after.
     * @see #addEdge(Enum, Enum, Command)
     */
    protected void addEdge(StateEnum start, StateEnum end) {
        addEdge(start, end, Commands.none());
    }

    /**
     * Add edges from all start states to the end state.
     * @param start The list of start states.
     * @param end The end state.
     * @param command The edge command.
     * @see #addEdge(Enum, Enum, Command)
     */
    protected void addMultiEdge(List<StateEnum> start, StateEnum end, Supplier<Command> command) {
        for(StateEnum state : start){
            if(state != end) {
                addEdge(state, end, command.get());
            }
        }
    }

    /**
     * Add edges from all start states to the end state.
     * @param start The list of start states.
     * @param end The end state.
     * @see #addEdge(Enum, Enum, Command)
     * @see #addEdge(Enum, Enum)
     */
    protected void addMultiEdge(List<StateEnum> start, StateEnum end) {
        addMultiEdge(start, end, Commands::none);
    }

    /**
     * Add edges from all states to the end state.
     * @param end The end state.
     * @param command The edge command.
     * @see #addEdge(Enum, Enum, Command)
     */
    protected void addOmniEdge(StateEnum end, Supplier<Command> command) {
        for(StateEnum state : stateEnumClass.getEnumConstants()) {
            if(state != end) {
                addEdge(state, end, command.get());
            }
        }
    }

    /**
     * Add the same edge for the transition from state1 to state2, and for the transition from state2 to state1.
     * @param state1 A state.
     * @param state2 Another state.
     * @param command The edge command. It is a supplier instead of a command so each edge will get a duplicate of the command to avoid errors.
     * @see #addEdge(Enum, Enum, Command)
     */
    protected void addCommutativeEdge(StateEnum state1, StateEnum state2, Supplier<Command> command) {
        addEdge(state1, state2, command.get());
        addEdge(state2, state1, command.get());
    }

    /**
     * Add the same edge for the transition from state1 to state2, and for the transition from state2 to state1.
     * @param state1 A state.
     * @param state2 Another state.
     * @see #addEdge(Enum, Enum, Command)
     * @see #addEdge(Enum, Enum)
     */
    protected void addCommutativeEdge(StateEnum state1, StateEnum state2) {
        addEdge(state1, state2);
        addEdge(state2, state1);
    }

    /**
     * Add an end condition for a state.
     * When the command finishes running, the statemachine will fire the transition command between the state to the wanted state.
     * The commands MUST NOT convey actual logic, but should only be for waiting for some event.
     * For example, any waitUntil or waitTime commands are accepted.
     * @param state The state this end condition applies to.
     * @param nextStatesMap A map of commands to states where each command represents an end condition which when finishes will transition to the matching state.
     */
    protected void addStateEnd(StateEnum state, Map<Command, StateEnum> nextStatesMap) {
        stateEnds.put(state, nextStatesMap);
    }

    /**
     * Prints the statemachine graph including states and edges to the console.
     */
    public void printGraph() {
        System.out.println("---------------StateMachine Graph---------------");
        for (Object vertex : instance.graph.vertexSet()) {
            System.out.println("Vertex: " + vertex);

            var outgoingEdges = graph.outgoingEdgesOf((StateEnum) vertex);
            if (outgoingEdges.isEmpty()) {
                System.out.println("  (no outgoing edges)");
            } else {
                for (Command edge : outgoingEdges) {
                    StateEnum target = graph.getEdgeTarget(edge);
                    System.out.println("  → " + target);
                }
            }
            System.out.println();
        }
        System.out.println("---------------StateMachine Graph---------------");
    }
}
