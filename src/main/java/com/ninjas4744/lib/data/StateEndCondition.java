package com.ninjas4744.lib.data;


import java.util.function.BooleanSupplier;

@SuppressWarnings({ "unchecked", "rawtypes" })
public class StateEndCondition<RobotStates extends Enum> {
	public BooleanSupplier condition;
	public RobotStates nextState;

	public StateEndCondition(BooleanSupplier condition, RobotStates nextState) {
		this.condition = condition;
		this.nextState = nextState;
	}
}
