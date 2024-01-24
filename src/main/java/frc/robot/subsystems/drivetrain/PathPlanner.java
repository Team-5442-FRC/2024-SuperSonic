// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;

public class PathPlanner extends SubsystemBase {
  /** Creates a new PathPlanner. */
  public PathPlanner() {}

  public PathPlannerPath getToPoint(Pose2d initPose, Pose2d desiredPose) {
    List<Translation2d> pathPoints = PathPlannerPath.bezierFromPoses(
      initPose,
      desiredPose
    );
    PathPlannerPath path = new PathPlannerPath(pathPoints,
      new PathConstraints(driveConstants.MaxSpeed, driveConstants.MaxAcceleration, driveConstants.MaxAngularRate, driveConstants.MaxAngularAcceleration),
      new GoalEndState(0.0, desiredPose.getRotation())
    );
    path.preventFlipping = true;
    return path;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
