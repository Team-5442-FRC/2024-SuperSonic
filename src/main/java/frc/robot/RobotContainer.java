// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.PathPlanner;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.CenterFunnyun;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.NoFunnyun;
import frc.robot.commands.ResetClimberCoder;
import frc.robot.commands.ShooterManager;
import frc.robot.commands.ToggleClimberLimits;

public class RobotContainer {

  public static PIDController speakerPID = shooterConstants.speakerPID; // PID loop for speaker auto rotate

  public static Pigeon2 pigeon2;
  public static TalonFX dFR, dFL, dBL, dBR;
  public static CANcoder cFR, cFL, cBL, cBR;


  // /* Setting up bindings for necessary control of the swerve drive platform */

  public final static CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public final static SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric() //Field oriented drive
    .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage)
    .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Adds a deadzone to the robot's speed
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public final static SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric() //Field oriented drive
    .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage)
    .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Adds a deadzone to the robot's speed
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  public final static Telemetry logger = new Telemetry(driveConstants.MaxSpeed);
  
  public static Vision vision = new Vision();
  public static Odometry odometry;
  public static PathPlanner pathplanner = new PathPlanner();

  //// Pivor \\\\\
  public static PIDController chassisPID;
  public static boolean shootOverride = false;
  public static boolean revOverride = false;
  public static boolean intakeOverride = false;

  ///// OPERATOR CONTROLLER \\\\\
  public static XboxController xbox2 = new XboxController(1); // Operator joystick
  public static XboxController xbox1 = new XboxController(0);
  public static JoystickButton xbox2A = new JoystickButton(xbox2, 1);
  public static JoystickButton xbox2B = new JoystickButton(xbox2, 2);
  public static JoystickButton xbox2Y = new JoystickButton(xbox2, 4);


  ///// ARM VARIABLES \\\\\
  public static boolean hasFunnyun = false;
  public static Command gotFunnyun, noFunnyun;
  public static Shooter shooter;
  public static DutyCycleEncoder pivotCoder;
  public static DigitalInput FrontProximitySensor = new DigitalInput(0);
  public static DigitalInput BackProximitySensor = new DigitalInput(1);//Electric Boogaloo

  public static CANSparkMax leftPivotMotor, rightPivotMotor; // Motor connected to arm pivot gearbox
  public static CANSparkFlex shooterMotor1, shooterMotor2; // Shoot motor closest to metal frame - blue wheels
  public static CANSparkFlex intakeMotor; // Shoot motor on the bottom - orange wheels connected with belt
  public static int ShooterMode = 0; //0 is intake, 1 is speaker, and 2 is amp. 
  static ShooterManager setShooterSpeed;
  public static CANdle led = new CANdle(0);

  ///// CLIMBER \\\\\
  public static CANSparkMax climberMotor;
  public static DutyCycleEncoder climberCoder;
  public static boolean climberLimits = true;
  public static Climber climber;
  static ShooterManager shooterManager;

  public static Command ShootCargo, Shoot, Intake, kShootOverride;



  private void configureBindings() {



    // /*-------------- CTRE CODE START ----------------*/


      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() -> 
              driveField
              .withVelocityX(-Math.pow(Deadzone(joystick.getLeftY()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive forward with negative Y (forward)
              .withVelocityY(-Math.pow(Deadzone(joystick.getLeftX()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive left with negative X (left)
              .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));



      // while (ShooterMode == 1){// && vision.isFacingSpeaker()) { // Speaker auto target
      //     drivetrain.applyRequest(() -> 
      //         driveField
      //         .withVelocityX(-Math.pow(Deadzone(joystick.getLeftY()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive forward with negative Y (forward)
      //         .withVelocityY(-Math.pow(Deadzone(joystick.getLeftX()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive left with negative X (left)
      //         .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate
      //         + speakerPID.calculate(vision.angleDifference)) // Drive counterclockwise with negative X (left) AND AUTOTARGET
      //   );
      // }



      joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

      joystick.rightBumper().whileTrue(
          drivetrain.applyRequest(() ->
          driveRobot
              .withVelocityX(Math.pow(Deadzone(joystick.getLeftY()), 3) * driveConstants.MaxSpeed) //* (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive forward with negative Y (forward)
              .withVelocityY(Math.pow(Deadzone(joystick.getLeftX()), 3) * driveConstants.MaxSpeed) //* (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive left with negative X (left)
              .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

      // reset the field-centric heading on down dpad press (used to be left bumper)
      // joystick.leftBumper().onTrue(new Command() {
      joystick.povDown().onTrue(new Command() {
        @Override
        public void initialize() {
          drivetrain.seedFieldRelative();
        }
        @Override 
        public boolean isFinished() {
          return true;
        }
      });

      if (Utils.isSimulation()) {
        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      }

      drivetrain.registerTelemetry(logger::telemeterize);

      


    // /*-------------- CTRE CODE END ----------------*/

    gotFunnyun = new ParallelRaceGroup(
      new CenterFunnyun(shooterConstants.ReverseIntakeAutomaticSpeed),
      new WaitCommand(shooterConstants.ReverseDuration)
    ); // Shoves the funnyun back after recieving it. 

    noFunnyun = new SequentialCommandGroup(
      new WaitCommand(0.5),
      new NoFunnyun()
    ); // Waits until after the funnyun has cleared the flywheels to report that there is no longer a funnyun
    

    shooter.setDefaultCommand(shooterManager);
    climber.setDefaultCommand(new ClimbCommand());

    
  }
    
    
    //What we want
    // A Button - Fire command - shoot the funnyun if ready
    // Right Trigger - Rev button, aim the shooter if in speaker mode
    // (Speaker Mode / Amp Mode), two buttons to set it to either fire into the amp or the speaker.

  public static double Deadzone(double speed) {
    if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
    return 0;
  }

  public static void driveChassisSpeeds(ChassisSpeeds speeds) {
    drivetrain.setControl(
      driveRobot
      .withVelocityX(speeds.vxMetersPerSecond)
      .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond)
    );
  }

  public RobotContainer() {

    dFR = new TalonFX(1);
    dFL = new TalonFX(3);
    dBL = new TalonFX(5);
    dBR = new TalonFX(7);

    cFR = new CANcoder(12);
    cFL = new CANcoder(9);
    cBL = new CANcoder(10);
    cBR = new CANcoder(11);

    //// Pivor \\\\\
    chassisPID = driveConstants.chassisPID;
    chassisPID.setTolerance(driveConstants.ChassisPidTolerence);

    ///// CLIMBER \\\\\
    climberCoder = new DutyCycleEncoder(4);
    climberMotor = new CANSparkMax(15 , MotorType.kBrushless);
    climber = new Climber();
    SmartDashboard.putData("Reset Climber", new ResetClimberCoder());
    SmartDashboard.putData("Toggle Robot Limits", new ToggleClimberLimits());

    ///// INPUTS \\\\\
    xbox2A = new JoystickButton(xbox2, 1);
    xbox2B = new JoystickButton(xbox2, 2);
    xbox2Y = new JoystickButton(xbox2, 4);
    
    ///// AUTO \\\\\
    pigeon2 = new Pigeon2(0);
      
    ///// ARM \\\\\
      
    pivotCoder = new DutyCycleEncoder(2);
    leftPivotMotor = new CANSparkMax(13, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(14, MotorType.kBrushless);
    shooterMotor1 = new CANSparkFlex(1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkFlex(2, MotorType.kBrushless);
    intakeMotor = new CANSparkFlex(3, MotorType.kBrushless);
    shooter = new Shooter();
      
    leftPivotMotor.setIdleMode(IdleMode.kBrake);
    rightPivotMotor.setIdleMode(IdleMode.kBrake); 
    shooterMotor1.setIdleMode(IdleMode.kBrake);
    shooterMotor2.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    odometry = new Odometry();
    shooterManager = new ShooterManager();

    kShootOverride = new Command() {
      WaitCommand kWait;

      @Override
      public void initialize() {
        kWait = new WaitCommand(0.5);
        kWait.schedule();
      }

      @Override
      public void end(boolean interrupted) {
        shootOverride = true;
      }

      @Override
      public boolean isFinished() {
        return kWait.isFinished();
      }
    };

    ShootCargo = new Command() {
      boolean isFinished = false;

      @Override
      public void initialize() {
        hasFunnyun = true;
        ShooterMode = 1;
        revOverride = true;
      }

      @Override
      public void execute() {
        if( Math.abs(shooter.calculatePivotAngle() - shooter.getAngle()) < pivotConstants.Tolerance) {
          kShootOverride.schedule();
        }
        if(!hasFunnyun) {
          shootOverride = false;
          revOverride = false;
          ShooterMode = 0;
          isFinished = true;
        }
      }

      @Override 
      public boolean isFinished() {
        return isFinished;
      }
    };

    Shoot = new Command() {
      boolean isFinished = false;

      @Override
      public void initialize() {
        ShooterMode = 1;
        revOverride = true;
      }

      @Override
      public void execute() {
        if( Math.abs(shooter.calculatePivotAngle() - shooter.getAngle()) < pivotConstants.Tolerance) {
          kShootOverride.schedule();
        }
        if(!hasFunnyun) {
          shootOverride = false;
          revOverride = false;
          ShooterMode = 0;
          isFinished = true;
        }
      }

      @Override 
      public boolean isFinished() {
        return isFinished;
      }
    };

    Intake = new Command() {

      @Override
      public void initialize() {
        intakeOverride = true;
      }

      @Override
      public void end(boolean interrupted) {
        intakeOverride = false;
      }
    };

    NamedCommands.registerCommand("Shoot Cargo", ShootCargo);
    NamedCommands.registerCommand("Shoot", Shoot);
    NamedCommands.registerCommand("Intake", Intake);

    

    AutoBuilder.configureHolonomic(
      odometry::getPose,
      odometry::resetPose,
      odometry::getChassisSpeeds,
      RobotContainer::driveChassisSpeeds,
      driveConstants.config,
      
      () -> { //Flip the path if opposite team?
        return false;
      },
      RobotContainer.drivetrain
    );

    configureBindings();
  }

  public Command getAutonomousCommand() { 
    return pathplanner.getAuto();
  }
}
