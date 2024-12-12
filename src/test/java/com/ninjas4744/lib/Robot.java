package com.ninjas4744.lib;// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants;
import com.ninjas4744.NinjasLib.RobotStateIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

  public class RobotState extends RobotStateIO<st>{

  }

  public Robot() {
//    MainControllerConstants c = new MainControllerConstants();
//    c.main.id = 30;
//    c.controlConstants = ControlConstants.createTorqueCurrent(7.5, 0);
//    _shooter = new NinjasTalonFXController(c);
//
//    _controller = new CommandPS5Controller(0);
//    _controller.cross().whileTrue(Commands.startEnd(() -> _shooter.setVelocity(100), () -> _shooter.stop()));

     RobotStateIO.setInstance(new RobotState());

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
    System.out.println(VisionIO.getInstance().getVisionEstimations()[0].closestTagDist);
//    for(VisionOutput o : VisionIO.getInstance().getVisionEstimations()){
//    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

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
