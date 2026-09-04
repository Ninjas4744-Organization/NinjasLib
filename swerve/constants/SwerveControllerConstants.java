package frc.lib.NinjasLib.swerve.constants;

import frc.lib.NinjasLib.controllers.constants.ControlConstants;

public class SwerveControllerConstants {
    /** Regular swerve constants */
    public SwerveConstants swerveConstants;

    /** Swerve auto driving PID constants */
    public ControlConstants drivePIDConstants = ControlConstants.createPID(0, 0, 0, 0);

    /** Swerve auto driving angle PID constants */
    public ControlConstants rotationPIDConstants = ControlConstants.createPID(0, 0, 0, 0);

    public SwerveControllerConstants withSwerveConstants(SwerveConstants swerveConstants) {
        this.swerveConstants = swerveConstants;
        return this;
    }

    public SwerveControllerConstants withDrivePID(ControlConstants drivePIDConstants) {
        this.drivePIDConstants = drivePIDConstants;
        return this;
    }

    public SwerveControllerConstants withRotationPID(ControlConstants rotationPIDConstants) {
        this.rotationPIDConstants = rotationPIDConstants;
        return this;
    }
}