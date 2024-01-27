// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  public static double pivotAngle;

  public static CANSparkMax lPivotMotor, rPivotMotor;
  public static CANSparkFlex shooterMotor1, shooterMotor2, intakeMotor;

  /** Creates a new Pivot. */
  public Shooter() {
    lPivotMotor = RobotContainer.lPivotMotor;
    rPivotMotor = RobotContainer.rPivotMotor;
    shooterMotor1 = RobotContainer.shooterMotor1;
    shooterMotor2 = RobotContainer.shooterMotor2;
    intakeMotor = RobotContainer.intakeMotor;
  }

  // public void pivot(double speed) {
  //   speed *= Constants.pivotConstants.MaxSpeed; // Limit max rotation speed
  //   RobotContainer.lPivotMotor.set(speed);
  //   RobotContainer.rPivotMotor.set(speed); // WARNING - may need to reverse speed depending on how the gearbox works.    
  // }

  public void setShooterSpeed(double topSpeed, double bottomSpeed) {
    shooterMotor1.set(topSpeed);
    shooterMotor2.set(bottomSpeed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Funnyun?", RobotContainer.hasFunnyun);
    SmartDashboard.putBoolean("Raw Proximity Sensor", RobotContainer.proximitySensor.get());
    SmartDashboard.putNumber("Shooter Wheel Speeds", shooterMotor1.getOutputCurrent());
  }
}
