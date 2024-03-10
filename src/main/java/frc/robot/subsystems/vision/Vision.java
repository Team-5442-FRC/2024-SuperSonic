// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Telemetry;

public class Vision extends SubsystemBase {

  boolean target;
  double latency;

  private Pose2d fieldPose, localPose;

  private long targetID;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable visionTable;

  Telemetry logger;

  /** Creates a new Vision. */
  public Vision() {
    visionTable = inst.getTable("limelight");

    fieldPose = new Pose2d();
    localPose = new Pose2d();
    logger = RobotContainer.logger;
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

  public double getLatency() {
    return this.latency;
  }
  
  public boolean isFacingSpeaker() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.get() == Alliance.Blue && (targetID == 7 || targetID == 8)) return true; // If on Blu and speaker tags in view
    else if(ally.get() == Alliance.Red && (targetID == 3 || targetID == 4)) return true; // If on Red and speaker tags in view
   
    return false; // Otherwise, return false
  }


  @Override
  public void periodic() {
    targetID = visionTable.getEntry("tid").getInteger(-1); // Get ID of current April Tag

    target = visionTable.getEntry("tv").getBoolean(false);

    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.get() == Alliance.Blue) visionTable.getEntry("priorityid").setInteger(7); // If on Blu and speaker tags in view
    else if(ally.get() == Alliance.Red) visionTable.getEntry("priorityid").setInteger(4); //  Red and speaker tags in view

    //Get robot relative positional data

    double[] targetRelative = visionTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    this.localPose = new Pose2d(
      new Translation2d(
        targetRelative[0],
        targetRelative[2]
      ),
      Rotation2d.fromDegrees(targetRelative[5])
    );

    this.target = (visionTable.getEntry("tv").getDouble(0) == 1);


    //Get field relative positional data

    double[] fieldRelative = visionTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);


    this.fieldPose = new Pose2d(
      new Translation2d(
        fieldRelative[0] + 0.34,
        fieldRelative[1] + 0.75 - 0.61
      ),
      Rotation2d.fromDegrees(fieldRelative[5])
    );
    

    this.latency = fieldRelative[6];

    // SmartDashboard.putNumber("Robot Field Position X", logger.getPose().getX());
    // SmartDashboard.putNumber("Robot Field Position Y",  logger.getPose().getY());
  }
}
