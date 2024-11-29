package com.ninjas4744.NinjasLib.DataClasses;

public class SwerveConstants {
    /** Whether to invert the robot gyro, always ensure gyro is CCW+ CW- */
    public boolean invertGyro;

    /** Whether to drive without module velocity PID control */
    public boolean openLoop;

    /** Distance between modules in forward axis */
    public double trackWidth;

    /** Distance between modules in side axis */
    public double wheelBase;

    /** Max speed the swerve could possibly drive */
    public double maxSpeed; // meters per second

    /** Max speed the swerve could possibly rotate */
    public double maxAngularVelocity;

    /** CanCoder Invert */
    public boolean canCoderInvert;

    /** Module Specific Constants */
    public SwerveModuleConstants[] moduleConstants;

    /** How much to multiply the simulation speed. Meters per 0.02s -> meters per 1s */
    public double simulationToRealSpeedConversion = 0.02; //

    /** Simulation robot acceleration */
    public double simulationAcceleration = 12;

    /** Simulation robot angle acceleration */
    public double simulationAngleAcceleration = 18;
}
