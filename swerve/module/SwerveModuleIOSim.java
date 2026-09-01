package frc.lib.NinjasLib.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.NinjasLib.swerve.SwerveUtils;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    public final int moduleNumber;

    private final SwerveModuleSimulation simulationModule;

    private final SimulatedMotorController.GenericMotorController steerMotor;
    private final SimulatedMotorController.GenericMotorController driveMotor;

    private final PIDController drivePID;
    private final PIDController steerPID;
    private Rotation2d lastAngle;
    private final double maxModuleSpeed;
    private SwerveModuleState desiredState = new SwerveModuleState();
    private boolean isOpenLoop;
    private boolean preventJittering;

    public SwerveModuleIOSim(SwerveConstants swerveConstants, SwerveModuleConstants constants, SwerveModuleSimulation simulationModule) {
        moduleNumber = constants.moduleNumber;
        maxModuleSpeed = swerveConstants.limits.maxSpeed;

        this.simulationModule = simulationModule;

        driveMotor = simulationModule.useGenericMotorControllerForDrive();
        steerMotor = simulationModule.useGenericControllerForSteer();

        drivePID = new PIDController(swerveConstants.modules.driveMotorConstants.real.control.controlConstants.P, swerveConstants.modules.driveMotorConstants.real.control.controlConstants.I, swerveConstants.modules.driveMotorConstants.real.control.controlConstants.D);
        drivePID.setIZone(swerveConstants.modules.driveMotorConstants.real.control.controlConstants.IZone);

        steerPID = new PIDController(swerveConstants.modules.steerMotorConstants.real.control.controlConstants.P, swerveConstants.modules.steerMotorConstants.real.control.controlConstants.I, swerveConstants.modules.steerMotorConstants.real.control.controlConstants.D);
        steerPID.setIZone(swerveConstants.modules.steerMotorConstants.real.control.controlConstants.IZone);

        lastAngle = simulationModule.getCurrentState().angle;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean preventJittering) {
        desiredState = SwerveUtils.optimizeModuleState(desiredState, simulationModule.getCurrentState().angle);
        this.desiredState = desiredState;
        this.isOpenLoop = isOpenLoop;
        this.preventJittering = preventJittering;

        //Drive
        if (isOpenLoop)
            driveMotor.requestVoltage(Volts.of(desiredState.speedMetersPerSecond / maxModuleSpeed * 12));
        else
            driveMotor.requestVoltage(Volts.of(drivePID.calculate(simulationModule.getCurrentState().speedMetersPerSecond, desiredState.speedMetersPerSecond)));

        //Angle
        Rotation2d angle = desiredState.angle;
        if (preventJittering) {
            // Prevent rotating module if speed is less than 1%. Prevents jittering.
            angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxModuleSpeed * 0.01)) ? lastAngle : desiredState.angle;
        }
        //Prevent jumping from -180 to 180
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double error = MathUtil.inputModulus(angle.getRadians() - simulationModule.getCurrentState().angle.getRadians(), -errorBound, errorBound);
        angle = Rotation2d.fromRadians(simulationModule.getCurrentState().angle.getRadians() + error);
        //Rotate
        steerMotor.requestVoltage(Volts.of(steerPID.calculate(simulationModule.getCurrentState().angle.getRadians(), angle.getRadians())));
        lastAngle = angle;
    }

    public SwerveModuleIOInputs update() {
        SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

        inputs.moduleNumber = moduleNumber;
        inputs.state = simulationModule.getCurrentState();
        inputs.desiredState = desiredState;
        inputs.position = new SwerveModulePosition(simulationModule.getDriveWheelFinalPosition().in(Radians) * simulationModule.config.WHEEL_RADIUS.in(Meters), inputs.state.angle);
        inputs.absolutePosition = Rotation2d.kZero;

        return inputs;
    }

    @Override
    public void periodic() {
        //Drive
        if (isOpenLoop)
            driveMotor.requestVoltage(Volts.of(desiredState.speedMetersPerSecond / maxModuleSpeed * 12));
        else
            driveMotor.requestVoltage(Volts.of(drivePID.calculate(simulationModule.getCurrentState().speedMetersPerSecond, desiredState.speedMetersPerSecond)));

        //Angle
        Rotation2d angle = desiredState.angle;
        if (preventJittering) {
            // Prevent rotating module if speed is less than 1%. Prevents jittering.
            angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxModuleSpeed * 0.01)) ? lastAngle : desiredState.angle;
        }
        //Prevent jumping from -180 to 180
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double error = MathUtil.inputModulus(angle.getRadians() - simulationModule.getCurrentState().angle.getRadians(), -errorBound, errorBound);
        angle = Rotation2d.fromRadians(simulationModule.getCurrentState().angle.getRadians() + error);
        //Rotate
        steerMotor.requestVoltage(Volts.of(steerPID.calculate(simulationModule.getCurrentState().angle.getRadians(), angle.getRadians())));
        lastAngle = angle;
    }
}
