package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jgrapht.Graph;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
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
                    if(end.isFinished() || !end.isScheduled()) {
                        System.out.println("[StateMachine] End condition met on state " + getCurrentState().name() + ", switching state to: " + ends.get(end).name());
                        changeRobotState(ends.get(end), false);
                        break;
                    }
                }
            }
        }

        // Check if transition command ended
        if(currentEdge != null && (currentEdge.isFinished() || !currentEdge.isScheduled())) {
            System.out.println("[StateMachine] Transition ended from " + getCurrentState().name() + " to " + getTargetState().name());

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

            System.out.println("[StateMachine] Transition started from " + getCurrentState().name() + " to " + getTargetState().name());
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

    public boolean isTransitioning() {
        return currentEdge != null;
    }

    public StateEnum getCurrentState() {
        return (StateEnum) RobotStateBase.getInstance().getRobotState();
    }

    public StateEnum getTargetState() {
        if(currentEdge != null)
            return graph.getEdgeTarget(currentEdge);
        return null;
    }

    public Command getCurrentTransitionCommand() {
        return currentEdge;
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

    protected void addMultiEdge(StateEnum end, Supplier<Command> command, StateEnum... start) {
        for(StateEnum state : start){
            if(state != end) {
                addEdge(state, end, command.get());
            }
        }
    }

    protected void addMultiEdge(StateEnum end, StateEnum... start) {
        addMultiEdge(end, Commands::none, start);
    }

    protected void addOmniEdge(StateEnum end, Supplier<Command> command) {
        for(StateEnum state : stateEnumClass.getEnumConstants()) {
            if(state != end) {
                addEdge(state, end, command.get());
            }
        }
    }

    protected void addCommutativeEdge(StateEnum start, StateEnum end, Supplier<Command> command) {
        addEdge(start, end, command.get());
        addEdge(end, start, command.get());
    }

    protected void addCommutativeEdge(StateEnum start, StateEnum end) {
        addEdge(start, end);
        addEdge(end, start);
    }

    protected void addCommutativeEdge(StateEnum start, StateEnum end, Command forward, Command backward) {
        addEdge(start, end, forward);
        addEdge(end, start, backward);
    }

    protected void addStateEnd(StateEnum state, Map<Command, StateEnum> nextStatesMap) {
        stateEnds.put(state, nextStatesMap);
    }

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
