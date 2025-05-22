package frc.lib.NinjasLib.dataclasses;

import frc.lib.NinjasLib.controllers.Controller;

public class SwerveModuleConstants<T extends Controller> {
    public int moduleNumber;
    public RealControllerConstants driveMotorConstants;
    public RealControllerConstants angleMotorConstants;
    public double maxModuleSpeed;
    public int canCoderID;
    public Class<T> driveControllerType;
    public Class<T> angleControllerType;
    public boolean enableLogging;
    public boolean invertCANCoder;
    public double CANCoderOffset;

    public SwerveModuleConstants(int moduleNumber, RealControllerConstants driveMotorConstants, RealControllerConstants angleMotorConstants, double maxModuleSpeed, int canCoderID, Class<T> driveControllerType, Class<T> angleControllerType, boolean enableLogging, boolean invertCANCoder, double CANCoderOffset) {
        this.moduleNumber = moduleNumber;
        this.driveMotorConstants = driveMotorConstants;
        this.angleMotorConstants = angleMotorConstants;
        this.maxModuleSpeed = maxModuleSpeed;
        this.canCoderID = canCoderID;
        this.driveControllerType = driveControllerType;
        this.angleControllerType = angleControllerType;
        this.enableLogging = enableLogging;
        this.invertCANCoder = invertCANCoder;
        this.CANCoderOffset = CANCoderOffset;
    }
}
