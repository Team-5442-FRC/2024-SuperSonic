// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.driveConstants;

public class RobotContainer {

  public static AHRS navx;
  public static Pigeon2 pigeon2;

  /* Setting up bindings for necessary control of the swerve drive platform */

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


  ///// OPERATOR CONTROLLER \\\\\
  public static XboxController xbox2 = new XboxController(1); // Operator joystick
  public static JoystickButton xbox2A;
  public static JoystickButton xbox2B;
  public static JoystickButton xbox2Y;


  ///// ARM VARIABLES \\\\\
  public static Pivot pivot;

  public static CANSparkMax lPivotMotor; // Motor connected to arm pivot gearbox
  public static CANSparkMax rPivotMotor; // Motor connected to arm pivot gearbox
  public static CANSparkMax topShootMotor; // Shoot motor closest to metal frame - blue wheels
  public static CANSparkMax midShootMotor; // Shoot motor in the middle - blue wheels
  public static CANSparkMax botShootMotor; // Shoot motor on the bottom - orange wheels connected with belt



  private void configureBindings() {



    /*-------------- CTRE CODE START ----------------*/


      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() -> 
              driveField
              .withVelocityX(-joystick.getLeftY() * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-joystick.getLeftX() * driveConstants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick.getRightX() * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
          ));

      joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

      joystick.rightBumper().whileTrue(
          drivetrain.applyRequest(() ->
          driveRobot
              .withVelocityX(-joystick.getLeftY() * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-joystick.getLeftX() * driveConstants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick.getRightX() * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

      // reset the field-centric heading on left bumper press
      joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

      if (Utils.isSimulation()) {
        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      }

      drivetrain.registerTelemetry(logger::telemeterize);


    /*-------------- CTRE CODE END ----------------*/


      



    
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
    ///// INPUTS \\\\\
    xbox2A = new JoystickButton(xbox2, 1);
    xbox2B = new JoystickButton(xbox2, 2);
    xbox2Y = new JoystickButton(xbox2, 4);
    
    ///// AUTO \\\\\
    pigeon2 = new Pigeon2(0);
    navx = new AHRS(Port.kMXP);

    AutoBuilder.configureHolonomic(

      logger::getPose,
      drivetrain::seedFieldRelative,
      this::getChassisSpeeds,
      this::driveChassisSpeeds,
      driveConstants.config,

      () -> { //Flip the path if opposite team?
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },

      drivetrain

    );



    ///// ARM \\\\\
    pivot = new Pivot();

    lPivotMotor = new CANSparkMax(13, MotorType.kBrushed);
    rPivotMotor = new CANSparkMax(14, MotorType.kBrushed);
    topShootMotor = new CANSparkMax(15, MotorType.kBrushless);
    midShootMotor = new CANSparkMax(16, MotorType.kBrushless);
    botShootMotor = new CANSparkMax(17, MotorType.kBrushless);



    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
