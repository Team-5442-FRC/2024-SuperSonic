// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.shooterConstants;

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

      if(RobotContainer.ShooterMode == 1) {
        RobotContainer.shooter.setShooterSpeed(RobotContainer.shooter.getSpeakerSpeed());
      } else if (RobotContainer.ShooterMode == 2) {
        RobotContainer.shooter.setShooterSpeed(shooterConstants.AmpSpeed);
      } else RobotContainer.shooter.setShooterSpeed(0);
      
    } else {
      RobotContainer.shooter.setShooterSpeed(0);
    }

    if(RobotContainer.xbox2A.getAsBoolean() && RobotContainer.hasFunnyun && (RobotContainer.xbox2.getRightTriggerAxis() > shooterConstants.TriggerDeadzone || RobotContainer.xbox2.getRightBumper()) && RobotContainer.shooter.shooterMotor1.getOutputCurrent() <= shooterConstants.MotorSettledAmps ) {
      speed = shooterConstants.IntakeSpeed;
    }

    if(!kIsFunnyun && RobotContainer.proximitySensor.get()) {
      RobotContainer.noFunnyun.schedule();
      kIsFunnyun = true;
    } else {
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

    // Pivot Code
    if (RobotContainer.xbox2.getPOV() == 0) { // Shoot - Up
      RobotContainer.ShooterMode = 1;
    } else if (RobotContainer.xbox2.getPOV() == 90) { // Amp - Right
      RobotContainer.ShooterMode = 2;
    } else if (RobotContainer.xbox2.getPOV() == 180) { // Intake - Down
      RobotContainer.ShooterMode = 0;
    }

    if(RobotContainer.ShooterMode == 0) {
      RobotContainer.shooter.pivotToAngle(pivotConstants.IntakeAngle); //Intake Mode 
    } else if (RobotContainer.ShooterMode == 2) {
      RobotContainer.shooter.pivotToAngle(pivotConstants.AmpAngle); //Amp Mode
    } else if (RobotContainer.ShooterMode == 1) {
      RobotContainer.shooter.pivotToAngle(RobotContainer.shooter.calculatePivotAngle());
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
