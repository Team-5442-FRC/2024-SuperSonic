// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

  public static DutyCycleEncoder pivotCoder;
  public static CANSparkMax leftPivotMotor, rightPivotMotor;
  public  CANSparkFlex shooterMotor1, shooterMotor2, intakeMotor;
  public static PIDController PID;
  public static double targetAngle;

  public static Translation3d robotFieldPosition, speakerPosition;

  
  /** Creates a new Pivot. */
  public Shooter() {
    leftPivotMotor = RobotContainer.leftPivotMotor;
    rightPivotMotor = RobotContainer.rightPivotMotor;
    shooterMotor1 = RobotContainer.shooterMotor1;
    shooterMotor2 = RobotContainer.shooterMotor2;
    intakeMotor = RobotContainer.intakeMotor;
    pivotCoder = RobotContainer.pivotCoder;
    PID = new PIDController(pivotConstants.kP, pivotConstants.kI, pivotConstants.kD);
    PID.setTolerance(pivotConstants.kTolerance); // Degrees
    targetAngle = pivotConstants.IntakeAngle;

    if(DriverStation.getAlliance() == Optional.of(Alliance.Blue)) {
      speakerPosition = shooterConstants.BlueTeamSpeaker;
    } else if(DriverStation.getAlliance() == Optional.of(Alliance.Red)) {
      speakerPosition = shooterConstants.RedTeamSpeaker;
    }   
  }
  
  public static double getAngle() {
    return (pivotCoder.getAbsolutePosition() - pivotConstants.EncoderOffset) * 360;
  }

  public void pivotToAngle(double angle) {
    if (targetAngle < pivotConstants.MinAngle) targetAngle = pivotConstants.MinAngle;
    else if (targetAngle > pivotConstants.MaxAngle) targetAngle = pivotConstants.MaxAngle;
    double kDifference = angle - getAngle();
    double pivotSpeed = -PID.calculate(kDifference);

    if (Math.abs(pivotSpeed) > pivotConstants.MaxSpeed) {
      pivotSpeed = (Math.abs(pivotSpeed) / pivotSpeed) * pivotConstants.MaxSpeed;
      RobotContainer.xbox2.setRumble(RumbleType.kLeftRumble, 0.5);  
    } else RobotContainer.xbox2.setRumble(RumbleType.kLeftRumble, 0);

    if (getAngle() >= pivotConstants.MaxAngle && pivotSpeed > 0) {
      pivotSpeed = 0;
      RobotContainer.xbox2.setRumble(RumbleType.kRightRumble, 0.5);  
    } else if (getAngle() <= pivotConstants.MinAngle && pivotSpeed < 0) {
      pivotSpeed = 0;
      RobotContainer.xbox2.setRumble(RumbleType.kRightRumble, 0.5);
    } else RobotContainer.xbox2.setRumble(RumbleType.kRightRumble, 0);

    leftPivotMotor.set(pivotSpeed);
    rightPivotMotor.set(-pivotSpeed);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor1.set(speed);
    shooterMotor2.set(speed - 0.08);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public double calculatePivotAngle() {

    double kY = speakerPosition.getZ() - robotFieldPosition.getZ(); //Refer  to angle calculation drawing for what these variables mean
    double kZ = robotFieldPosition.getDistance(speakerPosition);

    double kAngle1 = Math.acos(0.61 / kZ); //Temporary angle values as per drawing, 0.61 is 2ft in meters, or the distance from the pivot to the imaginary plane made between the two flywheels
    double kAngle2 = Math.asin(kY / kZ);

    return 270 - (kAngle1 + kAngle2);
  }

  public double calculateChassisAngle() { //what angle to point the shooter at
    double kX = speakerPosition.getX() - robotFieldPosition.getX();
    double kY = speakerPosition.getY() - robotFieldPosition.getY();

    return -Math.atan(kX / kY);
  }

  public double getSpeakerSpeed() { //Get the speed of the flywheels for the distance to the speaker
    double kDistance = Math.sqrt(Math.pow(robotFieldPosition.getDistance(speakerPosition),2) - 4) - shooterConstants.DistanceOffset; //Gets distance from Shooter to Speaker
    double kSpeed = kDistance * shooterConstants.DistanceMultiplier;

    if(kSpeed < shooterConstants.ShooterMinSpeed) {
      kSpeed = shooterConstants.ShooterMinSpeed;
    }
    if(kSpeed > shooterConstants.ShooterMaxSpeed) {
      kSpeed = shooterConstants.ShooterMaxSpeed;
    }

    return kSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Funnyun?", RobotContainer.hasFunnyun);
    SmartDashboard.putNumber("Speaker Robot Angle", calculateChassisAngle());
    
    robotFieldPosition = new Translation3d(RobotContainer.vision.getLocalPose().getX(), RobotContainer.vision.getLocalPose().getX(), shooterConstants.PivotY); 
  }
}
