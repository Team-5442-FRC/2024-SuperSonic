// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.driveConstants;

public class Odometry extends SubsystemBase {

  private static Translation2d FR = new Translation2d(driveConstants.ChassisModulePosX, driveConstants.ChassisModulePosY);
  private static Translation2d FL = new Translation2d(-driveConstants.ChassisModulePosX, driveConstants.ChassisModulePosY);
  private static Translation2d BL = new Translation2d(-driveConstants.ChassisModulePosX, -driveConstants.ChassisModulePosY);
  private static Translation2d BR = new Translation2d(driveConstants.ChassisModulePosX, -driveConstants.ChassisModulePosY);

  private static SwerveModulePosition FRpos;
  private static SwerveModulePosition FLpos;
  private static SwerveModulePosition BLpos;
  private static SwerveModulePosition BRpos;
  private static SwerveModulePosition[] swerveModulePositions;

  private static SwerveDriveKinematics kinematics;

  public static SwerveDrivePoseEstimator poseEstimator;
  private static Pose2d pose = new Pose2d();

  private final Field2d m_field = new Field2d();


  /** Creates a new Odometry. */
  public Odometry() {

    SmartDashboard.putData("Field", m_field);


    FRpos = RobotContainer.drivetrain.getModule(1).getPosition(true);
    FLpos = RobotContainer.drivetrain.getModule(0).getPosition(true);
    BLpos = RobotContainer.drivetrain.getModule(2).getPosition(true);
    BRpos = RobotContainer.drivetrain.getModule(3).getPosition(true);

    swerveModulePositions = new SwerveModulePosition[] {FRpos, FLpos, BLpos, BRpos};

    kinematics = new SwerveDriveKinematics(FR,FL,BL,BR);

    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      RobotContainer.drivetrain.getRotation3d().toRotation2d(),
      swerveModulePositions,
      new Pose2d(),
      VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(2)), // Vision Standard Deviation (Default 0.01 and 5deg)
      VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30)) // Drive Standard Deviation (Default 0.02 and 30deg)
    );

  }

  public Pose2d getPose() {
    return pose;
  }

  public void resetPose(Pose2d pose) {
    System.out.println("Reset Pose");
    poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation3d().toRotation2d(), swerveModulePositions, pose);
    RobotContainer.drivetrain.seedFieldRelative(pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
      RobotContainer.drivetrain.getModule(1).getCurrentState(),
      RobotContainer.drivetrain.getModule(2).getCurrentState(),
      RobotContainer.drivetrain.getModule(0).getCurrentState(),
      RobotContainer.drivetrain.getModule(3).getCurrentState()
    );
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Updates the Odometry

    FRpos = RobotContainer.drivetrain.getModule(1).getPosition(true);
    FLpos = RobotContainer.drivetrain.getModule(0).getPosition(true);
    BLpos = RobotContainer.drivetrain.getModule(2).getPosition(true);
    BRpos = RobotContainer.drivetrain.getModule(3).getPosition(true);

    swerveModulePositions = new SwerveModulePosition[] {FRpos, FLpos, BLpos, BRpos};

    pose = poseEstimator.update(RobotContainer.drivetrain.getRotation3d().toRotation2d(), swerveModulePositions);  

    if (RobotContainer.vision.target()) {
      poseEstimator.addVisionMeasurement(RobotContainer.vision.getFieldPose(), Timer.getFPGATimestamp());
    }

    SmartDashboard.putNumber("Robot Position X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Position Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Position Angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());


    m_field.setRobotPose(poseEstimator.getEstimatedPosition());
    // SmartDashboard.putNumber("Robot Position X Difference", RobotContainer.vision.getFieldPose().getX() - poseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("Robot Position Y Difference", RobotContainer.vision.getFieldPose().getY() - poseEstimator.getEstimatedPosition().getY());
  }
  
}
