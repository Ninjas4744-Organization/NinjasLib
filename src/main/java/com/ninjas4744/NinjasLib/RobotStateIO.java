package com.ninjas4744.NinjasLib;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class RobotStateIO<StateEnum> {
    protected static RobotStateIO _instance;

    public static RobotStateIO getInstance() {
        if(_instance == null)
            throw new RuntimeException("RobotStateIO not initialized. Initialize RobotStateIO by setInstance() first.");
        return _instance;
    }

    public static void setInstance(RobotStateIO instance, TimedRobot robot) {
        _instance = instance;
        _instance.robot = robot;
    }

    protected StateEnum robotState;

    protected TimedRobot robot;

    /**
     * @return State of the robot
     */
    public StateEnum getRobotState() {
        return robotState;
    }

    /**
     * Sets the state of the robot to the given state
     *
     * @param state - the state to set the robot state to
     */
    public void setRobotState(StateEnum state) {
        System.out.println("[Robot State Change] " + robotState.toString() + " -> " + state.toString());
        robotState = state;
        SmartDashboard.putString("Robot State", robotState.toString());
    }

    /**
     * @return Whether the robot is at simulation mode or deployed on a real robot
     */
    public boolean isSimulated() {
        return robot.isSimulation();
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
