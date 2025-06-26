package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveInput {
    private ChassisSpeeds chassisSpeeds;
    private boolean fieldRelative;

    public SwerveInput() {
        chassisSpeeds = new ChassisSpeeds();
        fieldRelative = false;
    }

    public SwerveInput(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        this.chassisSpeeds = chassisSpeeds;
        this.fieldRelative = fieldRelative;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public double getVx() {
        return chassisSpeeds.vxMetersPerSecond;
    }

    public double getVy() {
        return chassisSpeeds.vyMetersPerSecond;
    }

    public double getO() {
        return chassisSpeeds.omegaRadiansPerSecond;
    }
}
