package com.ninjas4744.NinjasLib.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class StateMachineMotoredSubsystem<StateEnum> extends StateMachineSubsystem<StateEnum> {
	protected NinjasController _controller;
	protected NinjasSimulatedController _simulatedController;

	public StateMachineMotoredSubsystem() {
		if (RobotStateIO.getInstance().isSimulated()) setSimulationController();
		else setController();
	}

	protected NinjasController controller() {
		if (RobotStateIO.getInstance().isSimulated()) return _simulatedController;
		else return _controller;
	}

	/**
	 * Set the real controller of the subsystem.
	 *
	 * <p>Implement controller in the _controller variable,
	 */
	protected abstract void setController();

	/**
	 * Set the simulated controller of the subsystem.
	 *
	 * <p>Implement controller in the _simulatedController for the simulated one.
	 *
	 * <p>The simulated controller is optional, only set it if code will be simulated.
	 */
	protected abstract void setSimulationController();

	public abstract void resetSubsystem();

	public abstract boolean isResetted();

	/**
	 * @return Whether the subsystem is homed/reseted/closed
	 */
	public boolean isHomed() {
		return controller().isHomed();
	}

	/**
	 * @return Whether the subsystem is at its PIDF goal
	 */
	public boolean atGoal() {
		return controller().atGoal();
	}

	/**
	 * Runs the motor at the given percent.
	 *
	 * @param percent - how much to power the motor between -1 and 1
	 * @return a command that runs that on start and stops to motor on end
	 */
	public Command runMotor(double percent) {
		return Commands.startEnd(
				() -> controller().setPercent(percent), () -> controller().stop(), this);
	}

	@Override
	public void periodic() {
		super.periodic();

		controller().periodic();
	}
}
