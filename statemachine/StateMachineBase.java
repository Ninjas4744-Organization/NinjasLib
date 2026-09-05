package frc.lib.NinjasLib.statemachine;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.commands.StateEndCommand;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.SimpleDirectedGraph;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Generic base class for building a robot state machine as a {@link SubsystemBase}. A state
 * machine over {@code StateEnum} is a directed graph whose vertices are the enum's constants and
 * whose edges are transition {@link Command}s; subclasses build this graph by overriding
 * {@link #define()} and calling {@link #addEdge}, {@link #addOmniEdge}, {@link #addStateEnd} and
 * {@link #addStateCommand}.
 * <p>
 * Once built, callers drive the state machine with {@link #changeState}, {@link #changeStateForce}
 * or {@link #forceState} for a single transition, or {@link #runStatesPath} to automatically hop
 * through several states via the shortest path to a target. {@link #periodic()} advances the
 * currently-running transition command, fires any configured state-end conditions, starts/stops
 * each state's background command, advances an in-progress path, and logs the state machine's
 * status via {@link NinjasLogger} - subclasses must ensure it is called every loop (which happens
 * automatically for a registered {@link SubsystemBase}).
 *
 * @param <StateEnum> The enum type enumerating every state this state machine can be in.
 */
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

    /**
     * Creates a new StateMachineBase, initializes the graph with all enum states as vertices, and calls {@link #define()}.
     *
     * @param states The enum class representing all possible states of this state machine.
     */
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

    /**
     * Called periodically. Checks state end conditions and handles transition command completion,
     * then logs state machine data via AdvantageKit.
     */
    @Override
    public void periodic() {
        // Check state ends for current state
        if(!isTransitioning()) {
            Map<Command, StateEnum> ends = stateEnds.get(getCurrentState());
            if(ends != null) {
                for(Command end : ends.keySet()) {
                    if(end.isFinished() && canTransitionTo(ends.get(end))) {
                        NinjasLogger.logEvent("[" + getName() + "] State end condition " + getCurrentState().name() + " -> " + ends.get(end).name());
                        changeState(ends.get(end));
                        break;
                    }
                }
            }
        }

        // Check if transition command ended
        if(currentEdge != null && (currentEdge.isFinished() || !currentEdge.isScheduled())) {

            NinjasLogger.logEvent("[" + getName() + "] Ended transition " + getCurrentState().name() + " -> " + getTargetState().name());

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

        NinjasLogger.log(getName() + "/State Machine/Is Transitioning", isTransitioning());
        NinjasLogger.log(getName() + "/State Machine/Current State", getCurrentState().name());
        NinjasLogger.log(getName() + "/State Machine/Target State", getTargetState() == null ? "N/A" : getTargetState().name());
        NinjasLogger.log(getName() + "/State Machine/Path States", currentPath == null ? new String[0] : currentPath.stream().map(Enum::name).toArray(String[]::new));
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

            Map<Command, StateEnum> currentEnds = stateEnds.get(getCurrentState());
            if (currentEnds != null) {
                for(Command end : currentEnds.keySet()) {
                    if (end.isScheduled() && !end.isFinished())
                        end.cancel();
                }
            }

            Map<Command, StateEnum> targetEnds = stateEnds.get(wantedState);
            if (targetEnds != null) {
                for(Command end : targetEnds.keySet()) {
                    CommandScheduler.getInstance().schedule(end);
                }
            }

            NinjasLogger.logEvent("[" + getName() + "] Force state " + getCurrentState().name() + " -> " + wantedState.name());
            currentState = wantedState;

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

            NinjasLogger.logEvent("[" + getName() + "] Started " + getCurrentState().name() + " -> " + getTargetState().name());
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
     * Tries to set the state of the robot to the given state even if already transitioning,
     * by the connecting edge between the current (or target) state and the wanted one.
     *
     * @param wantedState The state to change the robot state to.
     */
    public void changeStateForce(StateEnum wantedState) {
        changeState(wantedState, true, false, false);
    }

    /**
     * Immediately sets the robot state to the given state without running a transition command.
     * Cancels any currently running edge command and state end conditions.
     *
     * @param wantedState The state to force the robot into.
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
     * Wraps {@link #changeState(Enum)} in an instant command, for binding a state transition to a
     * trigger (e.g. a button or another command sequence) instead of calling it directly.
     *
     * @param wantedState The state to change to.
     * @return An instant command that runs {@link #changeState(Enum)}.
     */
    public Command changeStateCommand(StateEnum wantedState) {
        return Commands.runOnce(() -> changeState(wantedState));
    }

    /**
     * Wraps {@link #changeStateForce(Enum)} in an instant command, for binding a forced transition
     * to a trigger instead of calling it directly.
     *
     * @param wantedState The state to change to.
     * @return An instant command that runs {@link #changeStateForce(Enum)}.
     */
    public Command changeStateForceCommand(StateEnum wantedState) {
        return Commands.runOnce(() -> changeStateForce(wantedState));
    }

    /**
     * Wraps {@link #forceState(Enum)} in an instant command, for binding a forced state assignment
     * to a trigger instead of calling it directly.
     *
     * @param wantedState The state to force into.
     * @return An instant command that runs {@link #forceState(Enum)}.
     */
    public Command forceStateCommand(StateEnum wantedState) {
        return Commands.runOnce(() -> forceState(wantedState));
    }

    /**
     * @param start The start state.
     * @param end The end state.
     * @return Whether an edge command connects {@code start} directly to {@code end}.
     */
    public boolean canTransitionTo(StateEnum start, StateEnum end) {
        return graph.containsEdge(start, end);
    }

    /**
     * @param state The state to check.
     * @return Whether an edge command connects the current state directly to {@code state}.
     */
    public boolean canTransitionTo(StateEnum state) {
        return graph.containsEdge(getCurrentState(), state);
    }

    /**
     * @return Whether the state machine is currently running an edge (transition) command between
     *     two states, i.e. {@link #getCurrentState()} is not yet settled and
     *     {@link #getTargetState()} is non-null.
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
     * @return The state machine's current state. While transitioning, this is still the state
     *     being transitioned away from until the edge command finishes - see
     *     {@link #getTargetState()} for the destination.
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

    /**
     * @param states One or more states to check against.
     * @return Whether {@link #getCurrentState()} is one of the given states.
     */
    public boolean isInStates(StateEnum... states) {
        return Arrays.asList(states).contains(getCurrentState());
    }

    /**
     * @return Whether the state machine is currently following a multi-hop path started by
     *     {@link #runStatesPath(Enum)}.
     */
    public boolean isRunningPath() {
        return currentPath != null;
    }

    /**
     * @return The final destination state of the path started by {@link #runStatesPath(Enum)}, or
     *     {@code null} if no path is currently running.
     */
    public StateEnum getPathTarget() {
        if (currentPath != null)
            return currentPath.get(currentPath.size() - 1);
        return null;
    }

    /**
     * @return The states remaining in the current {@link #runStatesPath(Enum)} path, excluding the
     *     current state, or {@code null} if no path is currently running.
     */
    public List<StateEnum> getCurrentPath() {
        return currentPath;
    }

    /**
     * Set in this function the commands to run in state transitions.
     * And the state end conditions.
     *
     * @see #addEdge(Enum, Enum, Command)
     * @see #addStateEnd(Enum, StateEndCommand, Enum)
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
            NinjasLogger.logEvent("[" + getName() + "] Start and end state of an edge cannot be the same");
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
    protected void addStateEnd(StateEnum state, StateEndCommand waitCommand, StateEnum nextState) {
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
     * Add an end condition for a state.
     * When enough time passed since transitioning to this start, the statemachine will fire the transition command between the state to the wanted state.
     * @param state The state this end condition applies to.
     * @param time Amount of time to wait before transition.
     * @param nextState The state to transition to when command finishes.
     */
    protected void addStateEnd(StateEnum state, Time time, StateEnum nextState) {
        stateEnds.get(state).put(Commands.waitTime(time), nextState);
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
     * Prints every state and its outgoing edges to the console. Called automatically once at
     * construction (after {@link #define()}) so the state machine's full transition graph is
     * visible for debugging on startup.
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
