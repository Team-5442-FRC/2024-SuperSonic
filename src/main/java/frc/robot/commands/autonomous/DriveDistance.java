// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveDistance extends Command {
  double distance = 1; // Meters
  double startPosition = RobotContainer.odometry.getPose().getX();

  /** Creates a new DriveDistance. */
  public DriveDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrain.applyRequest(() -> 
      RobotContainer.driveRobot
      .withVelocityX(0.05) // Drive forward with negative Y (forward)
      .withVelocityY(0) // Drive left with negative X (left)
      .withRotationalRate(0) // Drive counterclockwise with negative X (left)
    );
    RobotContainer.shooter.intakeMotor.set(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.odometry.getPose().getX() - startPosition) >= distance);
  }
}
