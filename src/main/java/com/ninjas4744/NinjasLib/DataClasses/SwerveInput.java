package com.ninjas4744.NinjasLib.DataClasses;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveInput {
    private final ChassisSpeeds chassisSpeeds;
    private final String type;
    private final boolean fieldRelative;

    public SwerveInput(ChassisSpeeds chassisSpeeds, boolean fieldRelative, String type) {
        this.chassisSpeeds = chassisSpeeds;
        this.type = type;
        this.fieldRelative = fieldRelative;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public String getType() {
        return type;
    }
}
