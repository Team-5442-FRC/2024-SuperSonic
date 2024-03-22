// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Constants {

    public static final class driveConstants {

        public final static double MaxSpeed = 15; //Max targeted speed in M/S 
        public final static double MaxAngularRate = 1.5 * Math.PI * 3; //Max targeted rotations / second -- 3/4ths of a rotation for now
        public final static double MaxAcceleration = 2; //Max acceleration in M/s/s;
        public final static double MaxAngularAcceleration = 4 * Math.PI;
        public final static double SpeedDeadbandPercentage = 0; //Deadband or Deadzone of requested speed, as a percentage of the maximum speed;

        public final static double ControllerDeadzone = 0.15;

        public final static PIDController chassisPID = new PIDController(0.01, 0, 0);
        public final static double ChassisPidTolerence = Math.PI/180;

        public final static double ChassisModulePosX = 13.375; //Middle of robot to module left/right position
        public final static double ChassisModulePosY = 10.375; //Middle of robot to module front/back position

        public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            new PIDConstants(0.3,0.01,0),
            new PIDConstants(0.5,0.01,0.01),
            MaxSpeed,
            Math.sqrt((ChassisModulePosX * ChassisModulePosX) + (ChassisModulePosY * ChassisModulePosY)),
            new ReplanningConfig()
        );

    }

    public static final class shooterConstants {

        public final static double ShooterMaxSpeed = 0.68; // Was 0.8
        public final static double ShooterMinSpeed = 0.68; // Was 0.4
        public final static double PassingSpeed = 0.54; // Was 0.56
        public final static double ShooterBottomSpeedDifference = 0.06; //How much slower should the bottom speeds be (Was 0.08)
        public final static double AmpSpeed = 0.4; // Both motors same speed
        public final static double IntakeSpeed = 0.25;
        public final static double ReverseIntakeSpeed = -0.15;
        public final static double ReverseIntakeAutomaticSpeed = -0.125;
        public final static double ReverseDuration = 0.5; // Seconds
        public final static double TriggerDeadzone = 0.9;
        public final static double MotorSettledAmps = 22; // For revving motors (No longer used, now using timer 3/15/24)
        public final static double MotorSettledTime = 1; // Seconds

        public final static double AutonomousShootWait = 0.75; // Seconds

        public final static double VisionUpdateFrequency = 0.5; // Seconds

        // 1.803 meters to center of speaker
        public final static double PivotY = 2.051102; // - 0.28 + 0.15; // Height from pivot (hexshaft) to speaker; Meters
        public final static double kA = 0; // Quadratic x^2
        public final static double kB = 0; // Quadratic x, was 0.2
        public final static double kC = 0; // Quadratic constant, was -0.6

        public final static PIDController speakerPID = new PIDController(0.01, 0, 0);
        
        public final static double DistanceMultiplier = 0.4; // For increasing shoot speed over distance
        public final static double DistanceOffset = 2;

        // public final static int RedTeamSpeakerTag = 4;
        // public final static int BlueTeamSpeakerTag = 7;

        public final static Translation3d RedTeamSpeaker = new Translation3d(16.4650,  5.547868, 2.051102); // x was 16.579342, then 16.4084.  Now adjusted to match AprilTags directly.
        public final static Translation3d BluTeamSpeaker = new Translation3d(0.1524,   5.547868, 2.051102); // x was -0.0381
        
    }
    
    public static final class pivotConstants {
        
        public final static double MaxSpeed = 1; //Percentage (1.0 = 100%)
        public final static double Tolerance = 3;
        public final static double EncoderOffset = -33;
        public final static double MinAngle = -45; //Degrees
        public final static double MaxAngle = 60;

        public final static double AmpAngle = -45;
        public final static double IntakeAngle = 60;
        public final static double SpeakerAngle = 42;
        public final static double PassingAngle = 37;
        
        public final static double kP = 0.02; // Proportion
        public final static double kI = 0.0001; // Integral
        public final static double kD = 0; // Derivative
        public final static double kTolerance = 0.25; // Degrees
        public final static double PivotRadius = 0.61; // Meters

    }

    public static final class climberConstants {

        // public final static double MinCycles = 0; // Minimum amount of encoder cycles (360 degrees per cycle)
        // public final static double MinDegrees = 0; // Minimum value of encoder degrees
        // public final static double MaxCycles = 0; // Maximum amount of encoder cycles (360 degrees per cycle)
        // public final static double MaxDegrees = 0; // Maximum value of encoder degrees
        // public final static double CycleTolerance = 180; // How many degrees have to change to be considered a cycle
        public final static double MinDistance = -2; // DO NOT LEAVE IN THE LOWEST POSITION WHEN RESETTING THE ENCODER!
        public final static double MaxDistance = 22.4; // SHOULD BE 22.4

    }

    public static final class autonomousConstants {

    }

}
