package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class StateMachineBase<StateEnum extends Enum<StateEnum>> extends SubsystemBase {
    protected StateEnum currentState;

    private final Graph<StateEnum, Command> graph;
    private final Class<StateEnum> stateEnumClass;
    private Command currentEdge;

    private final Map<StateEnum, Map<Command, StateEnum>> stateEnds;
    private final Map<StateEnum, Command> stateCommands;
    private final BackgroundCommand stateCommand;

    private List<StateEnum> currentPath;
    private final BFSShortestPath<StateEnum, Command> bfs;

    public StateMachineBase(Class<StateEnum> states) {
        graph = new SimpleDirectedGraph<>(Command.class);

        stateEnds = new HashMap<>();
        stateEnumClass = states;
        for (StateEnum state : states.getEnumConstants()) {
            graph.addVertex(state);
            stateEnds.put(state, new HashMap<>());
        }

        stateCommands = new HashMap<>();
        stateCommand = new BackgroundCommand();

        bfs = new BFSShortestPath<>(graph);

        define();
        printGraph();
    }

    @Override
    public void periodic() {
        // Check state ends for current state
        if(!isTransitioning()) {
            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    if((end.isFinished() || !end.isScheduled()) && canTransitionTo(ends.get(end))) {
                        System.out.println("[" + getName() + "] State end condition " + getCurrentState().name() + " -> " + ends.get(end).name());
                        changeState(ends.get(end));
                        break;
                    }
                }
            }
        }

        // Check if transition command ended
        if(currentEdge != null && (currentEdge.isFinished() || !currentEdge.isScheduled())) {
            System.out.println("[" + getName() + "] Ended transition " + getCurrentState().name() + " -> " + getTargetState().name());

            currentState = getTargetState();
            currentEdge = null;

            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    CommandScheduler.getInstance().schedule(end);
                }
            }

            Command stateTask = stateCommands.get(getCurrentState());
            if (stateTask != null)
                stateCommand.setNewTask(stateTask);
            else
                stateCommand.stop();

            if (currentPath != null) {
                changeState(currentPath.get(0), false, false, true);

                currentPath.remove(0);
                if (currentPath.isEmpty())
                    currentPath = null;
            }
        }

        Logger.recordOutput(getName() + "/State Machine/Is Transitioning", isTransitioning());
        Logger.recordOutput(getName() + "/State Machine/Current State", getCurrentState());
        Logger.recordOutput(getName() + "/State Machine/Target State", getTargetState() == null ? "N/A" : getTargetState().name());
        Logger.recordOutput(getName() + "/State Machine/Path States", currentPath == null ? new String[0] : currentPath.stream().map(Enum::name).toArray(String[]::new));
    }

    /**
     * Tries to set the state of the robot to the given state by the connecting edge between the current state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     * @param forceTransition Whether to transition to a robot state even though the robot is currently already transitioning to a state.
     * @param forceState Whether to set the robot state to the wanted state no matter what. Doesn't run a transition command. Cancels current edge command and state ends.
     * @param fromPath Whether this was called from a state path. If it wasn't then deletes state path.
     */
    private void changeState(StateEnum wantedState, boolean forceTransition, boolean forceState, boolean fromPath) {
        if (!fromPath)
            currentPath = null;

        if (forceState) {
            if (currentEdge != null)
                currentEdge.cancel();

            currentEdge = null;
            System.out.println("[" + getName() + "] Force state " + getCurrentState().name() + " -> " + wantedState.name());
            currentState = wantedState;

            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    if (end.isScheduled() && !end.isFinished())
                        end.cancel();
                }
            }

            Command stateTask = stateCommands.get(getCurrentState());
            if (stateTask != null)
                stateCommand.setNewTask(stateTask);
            else
                stateCommand.stop();

            return;
        }

        if(isTransitioning() && !forceTransition)
            return;

        Command edge;
        if (isTransitioning() && forceTransition)
            edge = graph.getEdge(getTargetState(), wantedState);
        else
            edge = graph.getEdge(getCurrentState(), wantedState);

        if(edge != null) {
            if (currentEdge != null)
                currentEdge.cancel();

            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    end.cancel();
                }
            }

            stateCommand.stop();

            currentEdge = edge;
            CommandScheduler.getInstance().schedule(currentEdge);

            System.out.println("[" + getName() + "] Started " + getCurrentState().name() + " -> " + getTargetState().name());
        }
    }

    /**
     * Tries to set the state of the robot to the given state by the connecting edge between the current state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     */
    public void changeState(StateEnum wantedState) {
        changeState(wantedState, false, false, false);
    }

    /**
     * Tries to set the state of the robot to the given state by the connecting edge between the current state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     */
    public void changeStateForce(StateEnum wantedState) {
        changeState(wantedState, true, false, false);
    }

    /**
     * Tries to set the state of the robot to the given state by the connecting edge between the current state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     */
    public void forceState(StateEnum wantedState) {
        changeState(wantedState, false, true, false);
    }

    /**
     * Calculates BFS path from current state to wantedState and starts automatically transitioning between states to reach wantedState.
     * <br />Starts a transition immediately at call and starts the next transitions each time a transition ends.
     * <br />Stops when reached wantedState.
     * <br />If there is no path between currentState and wantedState, does nothing,
     * @param wantedState The target state of the path
     */
    public void runStatesPath(StateEnum wantedState) {
        GraphPath<StateEnum, Command> path = bfs.getPath(getCurrentState(), wantedState);
        if (path == null || isTransitioning())
            return;

        currentPath = path.getVertexList();
        currentPath.remove(0);
        if (currentPath.isEmpty()) {
            currentPath = null;
            return;
        }

        changeState(currentPath.get(0), false, false, true);

        currentPath.remove(0);
        if (currentPath.isEmpty())
            currentPath = null;
    }

    /**
     * @return Instant command that runs changeState.
     */
    public Command changeStateCommand(StateEnum wantedState) {
        return Commands.runOnce(() -> changeState(wantedState));
    }

    /**
     * @return Instant command that runs changeState.
     */
    public Command changeStateForceCommand(StateEnum wantedState) {
        return Commands.runOnce(() -> changeStateForce(wantedState));
    }

    /**
     * @return Instant command that runs changeState.
     */
    public Command forceStateCommand(StateEnum wantedState) {
        return Commands.runOnce(() -> forceState(wantedState));
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
    public Command getCurrentEdgeCommand() {
        return currentEdge;
    }

    /**
     * @return The current robot state from RobotStateBase.
     */
    public StateEnum getCurrentState() {
        return currentState;
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

    public boolean isRunningPath() {
        return currentPath != null;
    }

    public StateEnum getPathTarget() {
        if (currentPath != null)
            return currentPath.get(currentPath.size() - 1);
        return null;
    }

    public List<StateEnum> getCurrentPath() {
        return currentPath;
    }

    /**
     * Set in this function the commands to run in state transitions.
     * And the state end conditions.
     *
     * @see #addEdge(Enum, Enum, Command)
     * @see #addStateEnd(Enum, Command, Enum)
     * @see #addStateCommand(Enum, Command)
     */
    protected abstract void define();

    /**
     * Add an edge command connecting the start state to the end state in the statemachine's graph.
     * @param start The state the command will from.
     * @param end The state the robot will be when the command finishes.
     * @param command The transition command (Doesn't need to change the robot state, this is handled by the statemachine).
     */
    protected void addEdge(StateEnum start, StateEnum end, Command command) {
        if (start.equals(end)) {
            System.out.println("[" + getName() + "] Start and end state of an edge cannot be the same");
            return;
        }
        graph.addEdge(start, end, command);
    }

    /**
     * Add a connection from the start state to the end state in the statemachine's graph.
     * @param start The state to transition from.
     * @param end The state the robot will be after.
     */
    protected void addEdge(StateEnum start, StateEnum end) {
        addEdge(start, end, Commands.none());
    }

    /**
     * Add edges from all start states to all end states.
     * @param start The list of start states.
     * @param end The list of end states.
     * @param command The edge command.
     */
    protected void addEdge(List<StateEnum> start, List<StateEnum> end, Supplier<Command> command) {
        for(StateEnum startState : start) {
            for(StateEnum endState : end) {
                if(startState != endState) {
                    addEdge(startState, endState, command.get());
                }
            }
        }
    }

    /**
     * Add edges from all start states to all end states.
     * @param start The list of start states.
     * @param end The list of end states.
     */
    protected void addEdge(List<StateEnum> start, List<StateEnum> end) {
        for(StateEnum startState : start) {
            for(StateEnum endState : end) {
                if(startState != endState) {
                    addEdge(startState, endState, Commands.none());
                }
            }
        }
    }

    /**
     * Add edges from all start states to the end state.
     * @param start The list of start states.
     * @param end The end state.
     * @param command The edge command.
     */
    protected void addEdge(List<StateEnum> start, StateEnum end, Supplier<Command> command) {
        addEdge(start, List.of(end), command);
    }

    /**
     * Add edges from all start states to the end state.
     * @param start The list of start states.
     * @param end The end state.
     */
    protected void addEdge(List<StateEnum> start, StateEnum end) {
        addEdge(start, end, Commands::none);
    }

    /**
     * Add edges from start state to all end states.
     * @param start The start state.
     * @param end The list of end states.
     * @param command The edge command.
     */
    protected void addEdge(StateEnum start, List<StateEnum> end, Supplier<Command> command) {
        addEdge(List.of(start), end, command);
    }

    /**
     * Add edges from start state to all end states.
     * @param start The start state.
     * @param end The list of end states.
     */
    protected void addEdge(StateEnum start, List<StateEnum> end) {
        addEdge(start, end, Commands::none);
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
     * Add an end condition for a state.
     * When the command finishes running, the statemachine will fire the transition command between the state to the wanted state.
     * The command MUST NOT convey actual logic, but should only be for waiting for some event.
     * For example, any waitUntil or waitTime commands are accepted.
     * @param state The state this end condition applies to.
     * @param waitCommand A command which represents an end condition which when finishes will transition to the state.
     * @param nextState The state to transition to when command finishes.
     */
    protected void addStateEnd(StateEnum state, Command waitCommand, StateEnum nextState) {
        stateEnds.get(state).put(waitCommand, nextState);
    }

    /**
     * Add an end condition for a state.
     * When the condition is true, the statemachine will fire the transition command between the state to the wanted state.
     * @param state The state this end condition applies to.
     * @param condition An end condition which when equals true will transition to the state.
     * @param nextState The state to transition to when command finishes.
     */
    protected void addStateEnd(StateEnum state, BooleanSupplier condition, StateEnum nextState) {
        stateEnds.get(state).put(Commands.waitUntil(condition), nextState);
    }

    /**
     * Set a command to run when the robot is in the given state.
     * The command starts running when a transition has ended and the target state is the given one.
     * The command ends when starting a transition to another state.
     * If the command finished early it will NOT be run again in a loop.
     * @param state The state to set its command
     * @param command The command to run on the state
     */
    protected void addStateCommand(StateEnum state, Command command) {
        stateCommands.put(state, command);
    }

    /**
     * Prints the statemachine graph including states and edges to the console.
     */
    public void printGraph() {
        System.out.println("---------------" + getName() + " Graph---------------");
        for (StateEnum vertex : graph.vertexSet()) {
            System.out.println("State: " + vertex);

            var outgoingEdges = graph.outgoingEdgesOf(vertex);
            if (outgoingEdges.isEmpty()) {
                System.out.println("  (no outgoing edges)");
            } else {
                for (Command edge : outgoingEdges) {
                    StateEnum target = graph.getEdgeTarget(edge);
                    System.out.println("  -> " + target);
                }
            }
            System.out.println();
        }
        System.out.println("---------------" + getName() + " Graph---------------");
    }
}
