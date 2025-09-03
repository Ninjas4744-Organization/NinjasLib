package frc.lib.NinjasLib.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.NinjasLib.swerve.SwerveUtils;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    public final int moduleNumber;

    private final SwerveModuleSimulation simulationModule;

    private final SimulatedMotorController.GenericMotorController angleMotor;
    private final SimulatedMotorController.GenericMotorController driveMotor;

    private final PIDController drivePID;
    private final PIDController anglePID;
    private Rotation2d lastAngle;
    private final double maxModuleSpeed;

    public SwerveModuleIOSim(SwerveModuleConstants constants, SwerveModuleSimulation simulationModule) {
        moduleNumber = constants.moduleNumber;
        maxModuleSpeed = constants.maxModuleSpeed;

        this.simulationModule = simulationModule;

        driveMotor = simulationModule.useGenericMotorControllerForDrive();
        angleMotor = simulationModule.useGenericControllerForSteer();

        drivePID = new PIDController(constants.driveMotorConstants.real.controlConstants.P, constants.driveMotorConstants.real.controlConstants.I, constants.driveMotorConstants.real.controlConstants.D);
        drivePID.setIZone(constants.driveMotorConstants.real.controlConstants.IZone);

        anglePID = new PIDController(constants.angleMotorConstants.real.controlConstants.P, constants.angleMotorConstants.real.controlConstants.I, constants.angleMotorConstants.real.controlConstants.D);
        anglePID.setIZone(constants.angleMotorConstants.real.controlConstants.IZone);

        lastAngle = simulationModule.getCurrentState().angle;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveUtils.optimizeModuleState(desiredState, simulationModule.getCurrentState().angle);

        //Drive
        if (isOpenLoop) driveMotor.requestVoltage(Volts.of(desiredState.speedMetersPerSecond / maxModuleSpeed * 12));
        else
            driveMotor.requestVoltage(Volts.of(drivePID.calculate(simulationModule.getCurrentState().speedMetersPerSecond, desiredState.speedMetersPerSecond)));

        //Angle
        // Prevent rotating module if speed is less than 3%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxModuleSpeed * 0.03)) ? lastAngle : desiredState.angle;
        //Prevent jumping from -180 to 180
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double error = MathUtil.inputModulus(angle.getRadians() - simulationModule.getCurrentState().angle.getRadians(), -errorBound, errorBound);
        angle = Rotation2d.fromRadians(simulationModule.getCurrentState().angle.getRadians() + error);
        //Rotate
        angleMotor.requestVoltage(Volts.of(anglePID.calculate(simulationModule.getCurrentState().angle.getRadians(), angle.getRadians())));
        lastAngle = angle;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.ModuleNumber = moduleNumber;
        inputs.State = simulationModule.getCurrentState();
        inputs.Position = new SwerveModulePosition(simulationModule.getDriveWheelFinalPosition().in(Radians) * simulationModule.config.WHEEL_RADIUS.in(Meters), inputs.State.angle);
        inputs.AbsoluteAngle = Rotation2d.kZero;
    }
}
