package com.ninjas4744.NinjasLib.DataClasses;

import java.util.function.BooleanSupplier;

public class StateEndCondition<StateEnum> {
	public BooleanSupplier condition;
	public StateEnum nextState;

	public StateEndCondition(BooleanSupplier condition, StateEnum nextState) {
		this.condition = condition;
		this.nextState = nextState;
	}
}
