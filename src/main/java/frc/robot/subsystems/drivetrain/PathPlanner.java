// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.driveConstants;

public class PathPlanner extends SubsystemBase {
  /** Creates a new PathPlanner. */
  public static SendableChooser<String> autoChooser, funnyunChooser;

  public PathPlanner() {
    
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Blue Left", "Blue Left");
    autoChooser.addOption("Blue Middle", "Blue Middle");
    autoChooser.addOption("Blue Right", "Blue Right");
    autoChooser.addOption("Red Left", "Red Left");
    autoChooser.addOption("Red Middle", "Red Middle");
    autoChooser.addOption("Red Right", "Red Right");

    funnyunChooser = new SendableChooser<>();
    funnyunChooser.setDefaultOption("No Auto", "null");
    funnyunChooser.addOption("Shoot and Park Auto", " Park");
    funnyunChooser.addOption("One Funnyun Auto", " Single");
    funnyunChooser.addOption("Two Funnyun Auto", " Double");
    funnyunChooser.addOption("Shoot First Auto", " Shoot First");
    funnyunChooser.addOption("Two Funnyun Auto + Third Pickup", " Triple");
    funnyunChooser.addOption("ACCURACY", " Accuracy");

  }

  public Command getAuto() {
    if (funnyunChooser.getSelected() != "null") {
      return AutoBuilder.buildAuto(autoChooser.getSelected() + funnyunChooser.getSelected());
    }
    return null;
  }

  // public PathPlannerPath getToPoint(Pose2d initPose, Pose2d desiredPose) {
  //   List<Translation2d> pathPoints = PathPlannerPath.bezierFromPoses(
  //     initPose,
  //     desiredPose
  //   );
  //   PathPlannerPath path = new PathPlannerPath(pathPoints,
  //     new PathConstraints(driveConstants.MaxSpeed, driveConstants.MaxAcceleration, driveConstants.MaxAngularRate, driveConstants.MaxAngularAcceleration),
  //     new GoalEndState(0.0, desiredPose.getRotation())
  //   );
  //   path.preventFlipping = true;
  //   return path;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(funnyunChooser);
  }
}
