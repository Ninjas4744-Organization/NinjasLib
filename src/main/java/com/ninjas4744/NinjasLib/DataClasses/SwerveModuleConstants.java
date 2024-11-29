package com.ninjas4744.NinjasLib.DataClasses;

public class SwerveModuleConstants {
    public int moduleNumber;
    public MainControllerConstants driveMotorConstants;
    public MainControllerConstants angleMotorConstants;
    public double maxModuleSpeed;
    public int canCoderID;

    public SwerveModuleConstants(int moduleNumber, MainControllerConstants driveMotorConstants, MainControllerConstants angleMotorConstants, double maxModuleSpeed, int canCoderID){
        this.moduleNumber = moduleNumber;
        this.driveMotorConstants = driveMotorConstants;
        this.angleMotorConstants = angleMotorConstants;
        this.maxModuleSpeed = maxModuleSpeed;
        this.canCoderID = canCoderID;
    }
}
