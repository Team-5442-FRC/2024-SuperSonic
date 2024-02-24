// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.CenterFunnyun;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.NoFunnyun;
import frc.robot.commands.ResetClimberCoder;
import frc.robot.commands.ShooterManager;
import frc.robot.commands.ToggleClimberLimits;

public class RobotContainer {

  public static AHRS navx;
  public static Pigeon2 pigeon2;


  // /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric() //Field oriented drive

      .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage)
      .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Adds a deadzone to the robot's speed
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric() //Field oriented drive
  .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage)
  .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Adds a deadzone to the robot's speed
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  public final static Telemetry logger = new Telemetry(driveConstants.MaxSpeed);
  
  public static Vision vision = new Vision();

  //// Pivor \\\\\
  public static PIDController chassisPID;

  ///// OPERATOR CONTROLLER \\\\\
  public static XboxController xbox2 = new XboxController(1); // Operator joystick
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

  ///// CLIMBER \\\\\
  public static CANSparkMax climberMotor;
  public static DutyCycleEncoder climberCoder;
  public static boolean climberLimits = true;
  public static Climber climber;



  private void configureBindings() {



    // /*-------------- CTRE CODE START ----------------*/


      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() -> 
              driveField
              .withVelocityX(-Math.pow(Deadzone(joystick.getLeftY()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive forward with negative Y (forward)
              .withVelocityY(-Math.pow(Deadzone(joystick.getLeftX()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive left with negative X (left)
              .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));





      joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

      joystick.rightBumper().whileTrue(
          drivetrain.applyRequest(() ->
          driveRobot
              .withVelocityX(Math.pow(Deadzone(joystick.getLeftY()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive forward with negative Y (forward)
              .withVelocityY(Math.pow(Deadzone(joystick.getLeftX()), 3) * driveConstants.MaxSpeed * (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive left with negative X (left)
              .withRotationalRate(Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

      // reset the field-centric heading on left bumper press
      joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

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
    

    shooter.setDefaultCommand(new ShooterManager());
    climber.setDefaultCommand(new ClimbCommand());
    
    
    //What we want
    // A Button - Fire command - shoot the funnyun if ready
    // Right Trigger - Rev button, aim the shooter if in speaker mode
    // (Speaker Mode / Amp Mode), two buttons to set it to either fire into the amp or the speaker.


    
  }

  public static double Deadzone(double speed) {
    if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
    return 0;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(
      navx.getVelocityX(),
      navx.getVelocityY(),
      pigeon2.getAngularVelocityZDevice().getValueAsDouble()
    );
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    drivetrain.applyRequest(() -> 
              driveField
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withRotationalRate(speeds.omegaRadiansPerSecond)
    );
  }

  public RobotContainer() {

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
    navx = new AHRS(Port.kMXP);
    
    // AutoBuilder.configureHolonomic(
      
    //   logger::getPose,
    //   drivetrain::seedFieldRelative,
    //   this::getChassisSpeeds,
    //   this::driveChassisSpeeds,
    //   driveConstants.config,
      
    //   () -> { //Flip the path if opposite team?
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
      
    //   drivetrain
      
    // ); //TODO fix auto 
      
      
      
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

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
