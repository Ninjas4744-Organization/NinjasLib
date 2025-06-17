package frc.lib.NinjasLib.dataclasses;

import frc.lib.NinjasLib.controllers.Controller;

public class SwerveModuleConstants {
    public int moduleNumber;
    public ControllerConstants driveMotorConstants;
    public ControllerConstants angleMotorConstants;
    public double maxModuleSpeed;
    public int canCoderID;
    public Controller.ControllerType driveControllerType;
    public Controller.ControllerType angleControllerType;
    public boolean invertCANCoder;
    public double CANCoderOffset;

    public SwerveModuleConstants(int moduleNumber,
                                 ControllerConstants driveMotorConstants,
                                 ControllerConstants angleMotorConstants,
                                 double maxModuleSpeed,
                                 int canCoderID,
                                 Controller.ControllerType driveControllerType,
                                 Controller.ControllerType angleControllerType,
                                 boolean invertCANCoder,
                                 double CANCoderOffset) {
        this.moduleNumber = moduleNumber;
        this.driveMotorConstants = driveMotorConstants;
        this.angleMotorConstants = angleMotorConstants;
        this.maxModuleSpeed = maxModuleSpeed;
        this.canCoderID = canCoderID;
        this.driveControllerType = driveControllerType;
        this.angleControllerType = angleControllerType;
        this.invertCANCoder = invertCANCoder;
        this.CANCoderOffset = CANCoderOffset;
    }
}
