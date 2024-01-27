// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.Telemetry;

public class Vision extends SubsystemBase {

  boolean target;

  private Pose2d fieldPose, localPose;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable visionTable;

  // Telemetry logger;

  /** Creates a new Vision. */
  public Vision() {
    visionTable = inst.getTable("limelight");

    fieldPose = new Pose2d();
    localPose = new Pose2d();
    // logger = RobotContainer.logger;
  }

  public boolean target() {
    return this.target;
  }

  public Pose2d getFieldPose() {
    return this.fieldPose;
  }

  public Pose2d getLocalPose() {
    return this.localPose;
  }
  



  @Override
  public void periodic() {

    target = visionTable.getEntry("tv").getBoolean(false);

    //Get robot relative positional data

    double[] targetRelative = visionTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    this.localPose = new Pose2d(
      new Translation2d(
        targetRelative[0],
        targetRelative[2]
      ),
      Rotation2d.fromRadians(targetRelative[4])
    );

    this.target = (visionTable.getEntry("tv").getDouble(0) == 1);


    //Get field relative positional data

    double[] fieldRelative = visionTable.getEntry("botpose").getDoubleArray(new double[6]);

    this.fieldPose = new Pose2d(
      new Translation2d(
        fieldRelative[0],
        fieldRelative[1]
      ),
      Rotation2d.fromRadians(fieldRelative[4])
    );

    if(target) {
      // RobotContainer.drivetrain.addVisionMeasurement(fieldPose, logger.getTime());
    }

    // SmartDashboard.putNumber("Robot Field Position X", logger.getPose().getX());
    // SmartDashboard.putNumber("Robot Field Position Y",  logger.getPose().getY());
    SmartDashboard.putNumber("Robot Local Position X", getLocalPose().getX());
    SmartDashboard.putNumber("Robot Local Position Y", getLocalPose().getY());

  }
}
