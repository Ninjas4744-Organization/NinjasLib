package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Swerve extends SwerveIO {
    private final SwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    protected Swerve(SwerveConstants constants) {
        super(constants);

        _kinematics = constants.kinematics;

        _modules = new SwerveModule[] {
            new SwerveModule(constants.moduleConstants[0]),
            new SwerveModule(constants.moduleConstants[1]),
            new SwerveModule(constants.moduleConstants[2]),
            new SwerveModule(constants.moduleConstants[3])
        };

        resetModulesToAbsolute();

        Shuffleboard.getTab("Swerve").addNumber("Wanted Vx", () -> robotRelativeSpeeds.vxMetersPerSecond);
        Shuffleboard.getTab("Swerve").addNumber("Wanted Vy", () -> robotRelativeSpeeds.vyMetersPerSecond);
        Shuffleboard.getTab("Swerve").addNumber("Wanted V0", () -> robotRelativeSpeeds.omegaRadiansPerSecond);

        Shuffleboard.getTab("Swerve").addNumber("Current Vx", () -> getChassisSpeeds(false).vxMetersPerSecond);
        Shuffleboard.getTab("Swerve").addNumber("Current Vy", () -> getChassisSpeeds(false).vyMetersPerSecond);
        Shuffleboard.getTab("Swerve").addNumber("Current V0", () -> getChassisSpeeds(false).omegaRadiansPerSecond);
    }

    @Override
    public void drive(ChassisSpeeds drive, boolean fieldRelative) {
        this.robotRelativeSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotStateWithSwerve.getInstance().getGyroYaw()) : drive;
        setModuleStates(_kinematics.toSwerveModuleStates(robotRelativeSpeeds), _constants.openLoop);
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
        ChassisSpeeds speeds = _kinematics.toChassisSpeeds(getModuleStates());
        return fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(speeds, RobotStateWithSwerve.getInstance().getGyroYaw()) : speeds;
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
        RobotStateWithSwerve.getInstance().updateRobotPose(getModulePositions());
    }
}
