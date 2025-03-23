package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveConstants;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SwerveIO {
    private final SwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;
    private ChassisSpeeds _wantedSpeeds = new ChassisSpeeds();

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
    }

    @Override
    public void driveO(ChassisSpeeds robotRelativeSpeeds) {
        _wantedSpeeds = robotRelativeSpeeds;
        setModuleStates(_kinematics.toSwerveModuleStates(_wantedSpeeds), _constants.openLoop);
    }

    /**
     * Sets the modules to the given states
     * @param desiredStates The wanted state for each module
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, _constants.maxSpeed);
        for (SwerveModule module : _modules) 
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop);
    }

    /**
     * @return array of module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : _modules)
            states[module.moduleNumber] = module.getState();
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
        for (SwerveModule module : _modules) positions[module.moduleNumber] = module.getPosition();
        return positions;
    }

    /** Resets the swerve modules to their absolute encoders */
    public void resetModulesToAbsolute() {
        System.out.println("---------------Reseting modules to absolute---------------");
        for (SwerveModule module : _modules)
            module.resetToAbsolute();
        System.out.println("---------------Reseting modules to absolute---------------");
    }

    @Override
    public void periodic() {
        super.periodic();

        RobotStateWithSwerve.getInstance().updateRobotPose(getModulePositions());

        for (SwerveModule module : _modules)
            module.periodic();

        if(!_constants.enableLogging)
            return;

        Logger.recordOutput("Swerve/Current Velocity", getChassisSpeeds(true));
        Logger.recordOutput("Swerve/Wanted Velocity", ChassisSpeeds.fromRobotRelativeSpeeds(_wantedSpeeds, RobotStateWithSwerve.getInstance().getGyroYaw()));
    }
}
