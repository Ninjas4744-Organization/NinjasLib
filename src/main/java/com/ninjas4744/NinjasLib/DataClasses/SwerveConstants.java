package com.ninjas4744.NinjasLib.DataClasses;

public class SwerveConstants {
    /** Whether to drive without module velocity PID control */
    public boolean openLoop;

    /** Distance between modules in forward axis */
    public double trackWidth;

    /** Distance between modules in side axis */
    public double wheelBase;

    /** Max speed the swerve could possibly drive */
    public double maxSpeed;

    /** Max speed the swerve could possibly rotate */
    public double maxAngularVelocity;

    /** CanCoder Invert */
    public boolean canCoderInvert;

    /** Module Specific Constants */
    public SwerveModuleConstants[] moduleConstants;

    /** Simulation robot acceleration */
    public double simulationAcceleration;

    /** Simulation robot angle acceleration */
    public double simulationAngleAcceleration;
}
