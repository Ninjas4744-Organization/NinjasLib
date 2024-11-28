package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotState;

public class Swerve extends SwerveIO {
    private final SwerveModule[] _modules;

    public Swerve(SwerveConstants constants) {
        super(constants);

        _modules = new SwerveModule[] {
            new SwerveModule(0, ),
            new SwerveModule(1, ),
            new SwerveModule(2, ),
            new SwerveModule(3, )
        };
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotState.getInstance().getGyroYaw()) : drive);
        setModuleStates(swerveModuleStates, SwerveConstants.kOpenLoop);
    }

    /**
     * Sets the modules to the given states
     * @param desiredStates The wanted state for each module
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : _modules) mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
    }

    /**
     * @return array of module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : _modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
        return fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(
            SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates()), RobotState.getInstance().getGyroYaw())
            : SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : _modules) positions[mod.moduleNumber] = mod.getPosition();
        return positions;
    }

    /**
     * Resets the swerve modules to their absolute encoders
     */
    public void resetModulesToAbsolute() {
        System.out.println("---------------Reseting modules to absolute---------------");
        for (SwerveModule mod : _modules) mod.resetToAbsolute();
        System.out.println("---------------Reseting modules to absolute---------------");
    }

    @Override
    public void periodic() {
        super.periodic();
        RobotStateIO.getInstance().updateRobotPose(getModulePositions());
    }
}
