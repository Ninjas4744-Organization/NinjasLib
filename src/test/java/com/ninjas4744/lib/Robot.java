package com.ninjas4744.lib;// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ninjas4744.NinjasLib.DataClasses.*;
import com.ninjas4744.NinjasLib.RobotStateIO;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import java.io.IOException;
import java.util.List;
import java.util.Map;

public class Robot extends TimedRobot {
//  NinjasController _shooter;
//  CommandPS5Controller _controller;

    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try{
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }catch(IOException e){
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags) {
        AprilTagFieldLayout layout;

        layout = RobotStateIO.getInstance().getAlliance() == DriverStation.Alliance.Blue
          ? kBlueFieldLayout
          : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) layout.getTags().removeIf(tag -> ignoredTags.contains(tag.ID));

        return layout;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return getFieldLayout(List.of());
    }

    public enum st{
        hey
    }

    public class RobotState extends RobotStateWithSwerve<st> {

    }

    public class SwerveConstants {
        public static final double kSpeedFactor = 0.5;
        public static final double kRotationSpeedFactor = 0.25;
        public static final double kJoystickDeadband = 0.2;
        public static final boolean kInvertGyro = false;

        public static final com.ninjas4744.NinjasLib.DataClasses.SwerveConstants kSwerveConstants = new com.ninjas4744.NinjasLib.DataClasses.SwerveConstants();
        static{
            kSwerveConstants.openLoop = true;
            kSwerveConstants.trackWidth = 0.62;
            kSwerveConstants.wheelBase = 0.62;

            kSwerveConstants.maxSpeed = 5;
            kSwerveConstants.maxAngularVelocity = 10.7;
            kSwerveConstants.simulationAcceleration = 12;
            kSwerveConstants.simulationAngleAcceleration = 18;

            kSwerveConstants.canCoderInvert = false;
            kSwerveConstants.moduleConstants = new SwerveModuleConstants[4];

            for(int i = 0; i < 4; i++){
                kSwerveConstants.moduleConstants[i] = new SwerveModuleConstants(i, new MainControllerConstants(), new MainControllerConstants(), kSwerveConstants.maxSpeed, 0);
                kSwerveConstants.moduleConstants[i].driveMotorConstants.main.inverted = true;
                kSwerveConstants.moduleConstants[i].driveMotorConstants.currentLimit = 50;
                kSwerveConstants.moduleConstants[i].driveMotorConstants.encoderConversionFactor = 0.0521545447;
                kSwerveConstants.moduleConstants[i].driveMotorConstants.subsystemName = "Swerve Module " + i + " Drive Motor";
                kSwerveConstants.moduleConstants[i].driveMotorConstants.createShuffleboard = false;
                kSwerveConstants.moduleConstants[i].angleMotorConstants.currentLimit = 50;
                kSwerveConstants.moduleConstants[i].angleMotorConstants.encoderConversionFactor = 28.125;
                kSwerveConstants.moduleConstants[i].angleMotorConstants.subsystemName = "Swerve Module " + i + " Angle Motor";
                kSwerveConstants.moduleConstants[i].angleMotorConstants.createShuffleboard = false;
                kSwerveConstants.moduleConstants[i].angleMotorConstants.controlConstants = ControlConstants.createPID(0.01, 0, 0.005, 0);
                kSwerveConstants.moduleConstants[i].maxModuleSpeed = kSwerveConstants.maxSpeed;
            }
            kSwerveConstants.moduleConstants[0].driveMotorConstants.main.id = 10;
            kSwerveConstants.moduleConstants[0].angleMotorConstants.main.id = 11;
            kSwerveConstants.moduleConstants[0].canCoderID = 40;

            kSwerveConstants.moduleConstants[1].driveMotorConstants.main.id = 12;
            kSwerveConstants.moduleConstants[1].angleMotorConstants.main.id = 13;
            kSwerveConstants.moduleConstants[1].canCoderID = 41;

            kSwerveConstants.moduleConstants[2].driveMotorConstants.main.id = 14;
            kSwerveConstants.moduleConstants[2].angleMotorConstants.main.id = 15;
            kSwerveConstants.moduleConstants[2].canCoderID = 42;

            kSwerveConstants.moduleConstants[3].driveMotorConstants.main.id = 16;
            kSwerveConstants.moduleConstants[3].angleMotorConstants.main.id = 17;
            kSwerveConstants.moduleConstants[3].canCoderID = 43;
        }

        public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
        static {
            kSwerveControllerConstants.swerveConstants = kSwerveConstants;
            kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(1, 0, 0, 0);
            kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(0.057, 0.09, 0.003, 10);
            kSwerveControllerConstants.axisLockPIDConstants = ControlConstants.createPID(0.14, 0, 0, 0);
            kSwerveControllerConstants.driveAssistThreshold = 2;
            kSwerveControllerConstants.driverFieldRelative = true;
            kSwerveControllerConstants.pathConstraints = new PathConstraints(5, 10, 8, 16);
            kSwerveControllerConstants.robotConfig = new RobotConfig(42, 6.883, new ModuleConfig(0.048, kSwerveConstants.maxSpeed, 1.2, DCMotor.getNEO(1), 50, 4), 0.62, 0.62);
        }

        public static final PathFollowingController kPathFollowingController =
          new PPHolonomicDriveController(
            new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
            new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
          );
    }


    public Robot() {
//    MainControllerConstants c = new MainControllerConstants();
//    c.main.id = 30;
//    c.controlConstants = ControlConstants.createTorqueCurrent(7.5, 0);
//    _shooter = new NinjasTalonFXController(c);
//
        CommandPS5Controller _controller = new CommandPS5Controller(0);
//    _controller.cross().whileTrue(Commands.startEnd(() -> _shooter.setVelocity(100), () -> _shooter.stop()));

        SwerveIO.setConstants(SwerveConstants.kSwerveConstants);
        SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());
        RobotStateWithSwerve.setInstance(new RobotState(), ((Swerve)SwerveIO.getInstance()).getKinematics(), false, (o) -> 0);
        RobotStateWithSwerve.getInstance().resetGyro(Rotation2d.k180deg);

        VisionConstants kVisionConstants = new VisionConstants();
        kVisionConstants.cameras = Map.of(
          "Front", new Transform3d(0.28 - 0.11 - 0.2, 0.105, -0.055, new Rotation3d(0, 30, 0)));

        kVisionConstants.maxAmbiguity = 0.2;
        kVisionConstants.maxDistance = 4;
        kVisionConstants.fieldLayoutGetter = this::getFieldLayout;

        VisionIO.setConstants(kVisionConstants);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();


//    _shooter.periodic();
//    System.out.println(VisionIO.getInstance().getVisionEstimations()[0].closestTagDist);
        for(VisionOutput o : VisionIO.getInstance().getVisionEstimations()){
            RobotStateWithSwerve.getInstance().updateRobotPose(o);
        }

        SwerveController.getInstance().periodic();
        SwerveIO.getInstance().periodic();

        SmartDashboard.putString("Swerve State", SwerveController.getInstance().getState().toString());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Commands.run(() -> {
            Translation2d pid = SwerveController.getInstance().pidTo(new Translation2d(3, 6));
            SmartDashboard.putNumber("pidx", pid.getX());
            SmartDashboard.putNumber("pidy", pid.getY());
            SwerveController.getInstance().setState(SwerveDemand.SwerveState.VELOCITY);
            SwerveController.getInstance()._demand.fieldRelative = true;
            SwerveController.getInstance()._demand.velocity = new ChassisSpeeds(pid.getX(), pid.getY(), 0);
        }).repeatedly().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
