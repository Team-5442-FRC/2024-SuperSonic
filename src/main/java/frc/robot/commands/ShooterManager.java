// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterManager extends Command {

  double speed;
  DigitalInput proximitySensor;
  boolean kIsFunnyun = true;

  /** Creates a new setShooterSpeed. */
  public ShooterManager() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    proximitySensor = RobotContainer.proximitySensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speed = 0;

    //Shooter Code

    if(RobotContainer.xbox2.getRightTriggerAxis() > shooterConstants.TriggerDeadzone && RobotContainer.hasFunnyun) {
        RobotContainer.shooter.setShooterSpeed(shooterConstants.ShooterTopSpeed, shooterConstants.ShooterBottomSpeed);
    } else {
      RobotContainer.shooter.setShooterSpeed(0,0);
    }

    if(RobotContainer.xbox2A.getAsBoolean() && RobotContainer.hasFunnyun && RobotContainer.xbox2.getRightTriggerAxis() > shooterConstants.TriggerDeadzone && RobotContainer.shooter.shooterMotor1.getOutputCurrent() <= shooterConstants.MotorSettledAmps ) {
      speed = shooterConstants.IntakeSpeed;

      if(!kIsFunnyun && RobotContainer.proximitySensor.get()) {
        RobotContainer.noFunnyun.schedule();
        kIsFunnyun = true;
      }
      kIsFunnyun = RobotContainer.proximitySensor.get();
    }

    //Intake Code
    if(RobotContainer.xbox2B.getAsBoolean() && !RobotContainer.hasFunnyun) {
      speed = shooterConstants.IntakeSpeed;
    } else if(RobotContainer.xbox2Y.getAsBoolean()) {
      speed = shooterConstants.ReverseIntakeSpeed;
    }

    RobotContainer.shooter.setIntakeSpeed(speed);

    if(RobotContainer.hasFunnyun && speed < 0) {
      RobotContainer.hasFunnyun = false;
    }

    if(speed > 0 && RobotContainer.hasFunnyun == false && !proximitySensor.get()) {
      RobotContainer.hasFunnyun = true;
      RobotContainer.gotFunnyun.schedule();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
