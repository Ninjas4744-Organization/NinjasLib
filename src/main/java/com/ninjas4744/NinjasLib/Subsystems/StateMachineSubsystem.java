package com.ninjas4744.NinjasLib.Subsystems;

import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

public abstract class StateMachineSubsystem<StateEnum> extends SubsystemBase {
	private final Map<StateEnum, Runnable> _periodicFunctionMap;
	private final Map<StateEnum, Runnable> _onChangeFunctionMap;
	private StateEnum previousRobotState;

	public StateMachineSubsystem() {
		_periodicFunctionMap = new HashMap<>();
		_onChangeFunctionMap = new HashMap<>();

		previousRobotState = (StateEnum) RobotStateIO.getInstance().getRobotState();

		setFunctionMaps();
	}

	/**
	 * Set in what state what function to run.
	 *
	 * <p>There is the periodic map that runs your function periodically every 20ms if the robot
	 * state is what you've chosen.
	 *
	 * <p>And there is also on change map that runs your function once on the moment the robot
	 * state changed to what you've chosen.
	 *
	 * <p>Examples:
	 *
	 * <p>addFunctionToOnChangeMap(() -> System.out.println("Started Intaking"), StateEnum.INTAKE);
	 * <p>addFunctionToPeriodicMap(() -> System.out.println("Intaking"), StateEnum.INTAKE);
	 *
	 * <p>Doing that will spam "Intaking" in the console when the robot is at INTAKE state and print
	 * "Started Intaking" at the moment the state changed to INTAKING.
	 *
	 * <p>Note: on change map functions always run before periodic map functions
	 *
	 * @see #addFunctionToOnChangeMap
	 * @see #addFunctionToPeriodicMap
	 */
	protected abstract void setFunctionMaps();

	/**
	 * adds a function to the function periodic map
	 * this function is called periodically
	 *
	 * @param function the function to add
	 * @param states the states that the function will run at
	 * @see #setFunctionMaps
	 */
	protected void addFunctionToPeriodicMap(Runnable function, StateEnum... states) {
		for (StateEnum state : states) _periodicFunctionMap.put(state, function);
	}

	/**
	 * adds a function to the function on change map
	 * this function is being called once upon detected
	 * change of current state to any of given states
	 *
	 * @param function the function to add
	 * @param states the states that the function will run at
	 * @see #setFunctionMaps
	 */
	protected void addFunctionToOnChangeMap(Runnable function, StateEnum... states) {
		for (StateEnum state : states) _onChangeFunctionMap.put(state, function);
	}

	@Override
	public void periodic() {
		if (!RobotStateIO.getInstance().getRobotState().equals(previousRobotState) && _onChangeFunctionMap.get(RobotStateIO.getInstance().getRobotState()) != null)
			_onChangeFunctionMap.get(RobotStateIO.getInstance().getRobotState()).run();
		previousRobotState = (StateEnum) RobotStateIO.getInstance().getRobotState();

		if(_periodicFunctionMap.get(RobotStateIO.getInstance().getRobotState()) != null)
			_periodicFunctionMap.get(RobotStateIO.getInstance().getRobotState()).run();
	}
}
