// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class RumbleDuration extends Command {

  WaitCommand wait = new WaitCommand(0);

  /** Creates a new RumbleDuration. */
  public RumbleDuration(double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    wait = new WaitCommand(seconds);
    wait.schedule();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 1);
    RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 0);
    RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wait.isFinished();
  }
}
