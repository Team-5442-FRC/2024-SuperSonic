// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Shooter;

// public class PivotCommand extends Command {
//   public double speed;

//   /** Creates a new PivotCommand. */
//   public PivotCommand() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.pivot);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     this.speed = RobotContainer.xbox2.getRawAxis(2);
//     RobotContainer.pivot.pivot(speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
