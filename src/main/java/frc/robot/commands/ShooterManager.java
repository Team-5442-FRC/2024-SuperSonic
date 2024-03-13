// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.shooterConstants;

public class ShooterManager extends Command {

  double speed;
  DigitalInput BackProximitySensor, FrontProximitySensor;
  boolean kIsFunnyun = true;
  boolean backTripped = false;
  double targetAngle = pivotConstants.IntakeAngle;
  double climberOffset = 0;

  /** Creates a new setShooterSpeed. */
  public ShooterManager() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    BackProximitySensor = RobotContainer.BackProximitySensor;
    FrontProximitySensor = RobotContainer.FrontProximitySensor;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(  ) {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speed = 0;

    //Shooter Code

    if((RobotContainer.xbox2.getRightTriggerAxis() > shooterConstants.TriggerDeadzone || RobotContainer.revOverride) && RobotContainer.hasFunnyun) {

      if(RobotContainer.ShooterMode == 1) {
        RobotContainer.shooter.setShooterSpeed(RobotContainer.shooter.getSpeakerSpeed());
      } else if (RobotContainer.ShooterMode == 2) {
        RobotContainer.shooter.setShooterSpeed(shooterConstants.AmpSpeed);
      } else if (RobotContainer.ShooterMode == 3) {
        RobotContainer.shooter.setShooterSpeed(shooterConstants.PassingSpeed);
      } else RobotContainer.shooter.setShooterSpeed(0);
      
    } else {
      RobotContainer.shooter.setShooterSpeed(0);
    }

    if((RobotContainer.xbox2A.getAsBoolean() || RobotContainer.shootOverride) && RobotContainer.hasFunnyun && (RobotContainer.xbox2.getRightTriggerAxis() > shooterConstants.TriggerDeadzone || RobotContainer.revOverride) && RobotContainer.shooter.shooterMotor1.getOutputCurrent() <= shooterConstants.MotorSettledAmps ) {
      speed = shooterConstants.IntakeSpeed;
    }

    if(!kIsFunnyun && RobotContainer.FrontProximitySensor.get() && (RobotContainer.xbox2.getRightTriggerAxis() > shooterConstants.TriggerDeadzone || RobotContainer.revOverride) && (RobotContainer.xbox2A.getAsBoolean() || RobotContainer.shootOverride)) {
      RobotContainer.noFunnyun.schedule();
      kIsFunnyun = true;
    } else {
      kIsFunnyun = !RobotContainer.FrontProximitySensor.get() || !RobotContainer.BackProximitySensor.get();
    }


    //Intake Code
    if((RobotContainer.xbox2B.getAsBoolean() || RobotContainer.intakeOverride) && !RobotContainer.hasFunnyun) {
      speed = shooterConstants.IntakeSpeed;
    } else if(RobotContainer.xbox2Y.getAsBoolean()) {
      speed = shooterConstants.ReverseIntakeSpeed;
    }

    RobotContainer.shooter.setIntakeSpeed(speed);

    if(RobotContainer.hasFunnyun && speed < 0) {
      RobotContainer.hasFunnyun = false;
    }

    if(RobotContainer.hasFunnyun) {
      RobotContainer.led.setLEDs(255,255,255);
    } else {
      RobotContainer.led.setLEDs(0, 0, 0);
    }

    // if(speed > 0 && RobotContainer.hasFunnyun == false && !BackProximitySensor.get() && !FrontProximitySensor.get()) {
    //   RobotContainer.hasFunnyun = true;
    //   RobotContainer.gotFunnyun.schedule();
    // }

        
    if (!BackProximitySensor.get() && FrontProximitySensor.get()) { // First stage of intake is tripped, not second stage
      backTripped = true;
    } else if (BackProximitySensor.get() && FrontProximitySensor.get()) { // Neither sensor is tripped
      backTripped = false;
    }

    if(speed > 0 // Driving
      && RobotContainer.hasFunnyun == false // Not currently registering a funnyun
      && !FrontProximitySensor.get() // Second stage of intake is tripped
      && backTripped) // First stage was tripped last cycle
    {
      RobotContainer.hasFunnyun = true;
      RobotContainer.gotFunnyun.schedule();
      new SequentialCommandGroup(new RumbleDuration(0.2), new WaitCommand(0.75), new RumbleDuration(0.2), new WaitCommand(0.2)).schedule();  
      backTripped = false;
    }


    // Pivot Code
    if (RobotContainer.xbox2.getPOV() == 0) { // Shoot - Up
      RobotContainer.ShooterMode = 1;
    } else if (RobotContainer.xbox2.getPOV() == 90) { // Amp - Right
      RobotContainer.ShooterMode = 2;
    } else if (RobotContainer.xbox2.getPOV() == 180) { // Intake - Down
      RobotContainer.ShooterMode = 0;
    } else if (RobotContainer.xbox2.getLeftBumper()) { // Passing - Left Bumper
      RobotContainer.ShooterMode = 3;
    } else if (RobotContainer.xbox2.getRightBumper()) { // For moving notes in auto
      RobotContainer.ShooterMode = 4;
    }

    if (RobotContainer.ShooterMode == 4) {
      RobotContainer.shooter.setShooterSpeed(0.2);
      RobotContainer.shooter.setIntakeSpeed(shooterConstants.IntakeSpeed);
    }

    // if (Math.abs(RobotContainer.xbox2.getLeftY()) >= shooterConstants.TriggerDeadzone) targetAngle += RobotContainer.xbox2.getLeftY();



    // targetAngle += RobotContainer.Deadzone(RobotContainer.xbox2.getLeftY());
    if(RobotContainer.ShooterMode == 0) {
      targetAngle = pivotConstants.IntakeAngle; //Intake Mode 
      climberOffset = 0;
    } else if (RobotContainer.ShooterMode == 2) {
      targetAngle = pivotConstants.AmpAngle + climberOffset; //Amp Mode
      climberOffset -= RobotContainer.Deadzone(RobotContainer.xbox2.getLeftY());
      // targetAngle += RobotContainer.xbox2.getLeftY();
    } else if (RobotContainer.ShooterMode == 1) {
      climberOffset = 0;
      targetAngle = RobotContainer.shooter.calculatePivotAngle(); //Speaker mode
    } else if (RobotContainer.ShooterMode == 3) {
      targetAngle = pivotConstants.PassingAngle; //Passing Mode
    }

    if (targetAngle >= pivotConstants.MaxAngle) {
      targetAngle = pivotConstants.MaxAngle;
    } else if (targetAngle <= pivotConstants.MinAngle) {
      targetAngle = pivotConstants.MinAngle;
    }

    if(climberOffset <= pivotConstants.MinAngle - pivotConstants.AmpAngle) {
      climberOffset = pivotConstants.MinAngle - pivotConstants.AmpAngle;
    } else if(climberOffset + pivotConstants.AmpAngle >= pivotConstants.MaxAngle ) {
      climberOffset = pivotConstants.MaxAngle - pivotConstants.AmpAngle;
    }

    RobotContainer.shooter.pivotToAngle(targetAngle);



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
