// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Climber Limits", RobotContainer.climberLimits);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    RobotContainer.shooterManager.schedule();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Optional<Alliance> ally = DriverStation.getAlliance();
    new Rotation2d();
    if(ally.get() == Alliance.Blue) { // Set blue field orientation.  TODO - CHECK IF THIS WORKS PROPERLY.  ADDED 3/7/24.
      RobotContainer.drivetrain.seedFieldRelative(new Pose2d(RobotContainer.odometry.getPose().getTranslation(), Rotation2d.fromDegrees(RobotContainer.odometry.getPose().getRotation().getDegrees())));
    }
    else if(ally.get() == Alliance.Red) { // Set red field orientation.  TODO - CHECK IF THIS WORKS PROPERLY.  ADDED 3/7/24.
      RobotContainer.drivetrain.seedFieldRelative(new Pose2d(RobotContainer.odometry.getPose().getTranslation(), Rotation2d.fromDegrees(360 - RobotContainer.odometry.getPose().getRotation().getDegrees())));
    }
  }

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

  @Override
  public void simulationPeriodic() {}
}
