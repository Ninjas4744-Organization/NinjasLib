package frc.lib.NinjasLib.dataclasses;

import java.util.function.BooleanSupplier;

public class StateEndCondition<StateEnum> {
	public BooleanSupplier condition;
	public StateEnum nextState;

	public StateEndCondition(BooleanSupplier condition, StateEnum nextState) {
		this.condition = condition;
		this.nextState = nextState;
	}
}
