// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
  public static double pivotAngle;

  public static CANSparkMax lPivotMotor, rPivotMotor;

  /** Creates a new Pivot. */
  public Pivot() {
    lPivotMotor = RobotContainer.lPivotMotor;
    rPivotMotor = RobotContainer.rPivotMotor;
  }

  public void pivot(float speed) {
    RobotContainer.lPivotMotor.set(speed);
    RobotContainer.rPivotMotor.set(speed); // WARNING - may need to reverse speed depending on how the gearbox works.    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
