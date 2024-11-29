package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Swerve extends SwerveIO {
    private final SwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;

    public Swerve(SwerveConstants constants) {
        super(constants);

        _kinematics = new SwerveDriveKinematics(
            new Translation2d(constants.wheelBase / 2.0, constants.trackWidth / 2.0),
            new Translation2d(constants.wheelBase / 2.0, -constants.trackWidth / 2.0),
            new Translation2d(-constants.wheelBase / 2.0, constants.trackWidth / 2.0),
            new Translation2d(-constants.wheelBase / 2.0, -constants.trackWidth / 2.0)
        );

        _modules = new SwerveModule[] {
            new SwerveModule(constants.moduleConstants[0]),
            new SwerveModule(constants.moduleConstants[1]),
            new SwerveModule(constants.moduleConstants[2]),
            new SwerveModule(constants.moduleConstants[3])
        };
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotStateIO.getInstance().getGyroYaw()) : drive);
        setModuleStates(swerveModuleStates, _constants.openLoop);
    }

    /**
     * Sets the modules to the given states
     * @param desiredStates The wanted state for each module
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, _constants.maxSpeed);

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
            _kinematics.toChassisSpeeds(getModuleStates()), RobotStateIO.getInstance().getGyroYaw())
            : _kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * @return driven distance and angle of each module
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : _modules) positions[mod.moduleNumber] = mod.getPosition();
        return positions;
    }

    /** Resets the swerve modules to their absolute encoders */
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
