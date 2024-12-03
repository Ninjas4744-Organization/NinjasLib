// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ninjas4744.lib;

import com.ninjas4744.NinjasLib.Controllers.NinjasController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class Robot extends TimedRobot {
  NinjasController _shooter;
  CommandPS5Controller _controller;

  public Robot() {
    MainControllerConstants c = new MainControllerConstants();
    c.main.id = 30;
    c.controlConstants = ControlConstants.createTorqueCurrent(7.5, 0);
    _shooter = new NinjasTalonFXController(c);

    _controller = new CommandPS5Controller(0);
    _controller.cross().whileTrue(Commands.startEnd(() -> _shooter.setVelocity(100), () -> _shooter.stop()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    _shooter.periodic();
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
