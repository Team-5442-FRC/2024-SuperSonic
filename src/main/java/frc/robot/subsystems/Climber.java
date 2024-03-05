// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {

  DutyCycleEncoder climberCoder;
  CANSparkMax climberMotor;

  // double lastEncoderValue;
  // int cycles = 0;
  
  /** Creates a new Climb. */
  public Climber() {

    climberCoder = RobotContainer.climberCoder;
    climberMotor = RobotContainer.climberMotor;

  }

  public void climb(double speed) {
    speed = -speed;


    if(speed > 0 && getClimberDistance() >= climberConstants.MaxDistance && RobotContainer.climberLimits) {
      RobotContainer.climberMotor.set(0);
    } else if (speed < 0 && getClimberDistance() <= climberConstants.MinDistance && RobotContainer.climberLimits) {
      RobotContainer.climberMotor.set(0);
    } else {
      RobotContainer.climberMotor.set(speed);
    }

    // if ((RobotContainer.climberCoder.get()) - lastEncoderValue >= climberConstants.CycleTolerance) { // Increase cycle counter
    //   cycles += 1;
    // } else if (lastEncoderValue - RobotContainer.climberCoder.get() >= climberConstants.CycleTolerance) { // Decrease
    //   cycles -= 1;
    // }

    // lastEncoderValue = RobotContainer.climberCoder.get();
  }

  public void resetClimberCoder() {
    climberCoder.reset();
  }

  public double getClimberDistance() {
    return -climberCoder.getDistance(); //Goes negative
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getClimberDistance());
  }
}
