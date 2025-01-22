package com.ninjas4744.NinjasLib.DataClasses;

import com.ninjas4744.NinjasLib.Controllers.NinjasController;

public class SwerveModuleConstants<T extends NinjasController> {
    public int moduleNumber;
    public MainControllerConstants driveMotorConstants;
    public MainControllerConstants angleMotorConstants;
    public double maxModuleSpeed;
    public int canCoderID;
    public Class<T> driveControllerType;
    public Class<T> angleControllerType;
    public boolean createShuffleboard;

    public SwerveModuleConstants(int moduleNumber, MainControllerConstants driveMotorConstants, MainControllerConstants angleMotorConstants, double maxModuleSpeed, int canCoderID, Class<T> driveControllerType, Class<T> angleControllerType, boolean createShuffleboard) {
        this.moduleNumber = moduleNumber;
        this.driveMotorConstants = driveMotorConstants;
        this.angleMotorConstants = angleMotorConstants;
        this.maxModuleSpeed = maxModuleSpeed;
        this.canCoderID = canCoderID;
        this.driveControllerType = driveControllerType;
        this.angleControllerType = angleControllerType;
        this.createShuffleboard = createShuffleboard;
    }
}
