package frc.lib.NinjasLib.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.Controllers.NinjasController;
import frc.lib.NinjasLib.Controllers.NinjasSimulatedController;
import frc.lib.NinjasLib.RobotStateIO;

public abstract class StateMachineMotoredSubsystem<StateEnum> extends StateMachineSubsystem<StateEnum> {
    protected NinjasController _controller;
    protected NinjasSimulatedController _simulatedController;

    public StateMachineMotoredSubsystem(boolean paused) {
        super(paused);

        if(_paused)
            return;

        if (RobotStateIO.isSimulated())
            setSimulationController();
        else
            setController();
    }

    protected NinjasController controller() {
        if (RobotStateIO.isSimulated())
            return _simulatedController;
        else
            return _controller;
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

    public void resetSubsystem(){
        if(!_paused)
            resetSubsystemO();
    }

    protected abstract void resetSubsystemO();

    public boolean isResetted(){
        if(!_paused)
            return isResettedO();
        return true;
    }

    protected abstract boolean isResettedO();

    /**
     * @return Whether the subsystem is at its PIDF goal
     */
    public boolean atGoal() {
        if(_paused)
            return true;

        return controller().atGoal();
    }

    /**
     * Runs the motor at the given percent.
     *
     * @param percent - how much to power the motor between -1 and 1
     * @return a command that runs that on start and stops to motor on end
     */
    public Command runMotor(double percent) {
        if(_paused)
            return Commands.none();

        return Commands.startEnd(
            () -> controller().setPercent(percent), () -> controller().stop());
    }

    @Override
    public void periodic() {
        if(_paused)
            return;

        super.periodic();
        controller().periodic();
    }
}
