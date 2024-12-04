package com.ninjas4744.NinjasLib;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public abstract class RobotStateIO<StateEnum> {
    protected static RobotStateIO _instance;
    protected StateEnum _robotState;
    protected TimedRobot _robot;

    public static RobotStateIO getInstance() {
        if(_instance == null)
            throw new RuntimeException("RobotStateIO not initialized. Initialize RobotStateIO by setInstance() first.");
        return _instance;
    }

    public static void setInstance(RobotStateIO instance, TimedRobot robot) {
        _instance = instance;
        _instance._robot = robot;
        _instance.init();
    }

    protected void init(){

    }

    /**
     * @return State of the robot
     */
    public StateEnum getRobotState() {
        return _robotState;
    }

    /**
     * Sets the state of the robot to the given state
     *
     * @param state - the state to set the robot state to
     */
    public void setRobotState(StateEnum state) {
        System.out.println("[Robot State Change] " + _robotState.toString() + " -> " + state.toString());
        _robotState = state;
    }

    /**
     * @return Whether the robot is at simulation mode or deployed on a real robot
     */
    public boolean isSimulated() {
        return _robot.isSimulation();
    }

    public boolean isAutonomous() {
        return isSimulated() ? DriverStationSim.getAutonomous() : DriverStation.isAutonomous();
    }

    public DriverStation.Alliance getAlliance() {
        return isSimulated()
            ? (DriverStationSim.getAllianceStationId().ordinal() > 3
            ? DriverStation.Alliance.Blue
            : DriverStation.Alliance.Red)
            : DriverStation.getAlliance().get();
    }

    public AllianceStationID getAllianceStation() {
        return isSimulated() ? DriverStationSim.getAllianceStationId() : DriverStation.getRawAllianceStation();
    }
}
