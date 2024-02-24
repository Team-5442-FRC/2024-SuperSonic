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

        public final static double MaxSpeed = 6; //Max targeted speed in M/S 
        public final static double MaxAngularRate = 1.5 * Math.PI; //Max targeted rotations / second -- 3/4ths of a rotation for now
        public final static double MaxAcceleration = 2; //Max acceleration in M/s/s;
        public final static double MaxAngularAcceleration = 4 * Math.PI;
        public final static double SpeedDeadbandPercentage = 0; //Deadband or Deadzone of requested speed, as a percentage of the maximum speed;

        public final static double ControllerDeadzone = 0.15;

        public final static PIDController chassisPID = new PIDController(0.01, 0, 0);
        public final static double ChassisPidTolerence = Math.PI/180;

        public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            new PIDConstants(0,0,0),
            new PIDConstants(0,0,0),
            0,
            0,
            new ReplanningConfig() //TODO set these properly
        );

    }

    public static final class shooterConstants {

        public final static double ShooterMaxSpeed = 1; // Was 0.8
        public final static double ShooterMinSpeed = 0.4; // Was 0.4
        public final static double ShooterBottomSpeedDifference = 0.08; //How much slower should the bottom speeds be
        public final static double AmpSpeed = 0.4; // Both motors same speed
        public final static double IntakeSpeed = 0.35;
        public final static double ReverseIntakeSpeed = -0.15;
        public final static double ReverseIntakeAutomaticSpeed = -0.1;
        public final static double ReverseDuration = 3; // Seconds
        public final static double TriggerDeadzone = 0.9;
        public final static double MotorSettledAmps = 18;

        public final static double VisionUpdateFrequency = 0.5; // Seconds

        public final static double PivotY = 2.051102 - 0.28 + 0.25; // Height from pivot (hexshaft) to speaker; Meters
        public final static double kA = 0; // Quadratic x^2
        public final static double kB = 0.2; // Quadratic x
        public final static double kC = -0.6; // Quadratic constant
        
        public final static double DistanceMultiplier = 0.4; // For increasing shoot speed over distance
        public final static double DistanceOffset = 2;

        // public final static int RedTeamSpeakerTag = 4;
        // public final static int BlueTeamSpeakerTag = 7;

        public final static Translation3d RedTeamSpeaker = new Translation3d(8.062467,  1.442593, 2.051102);
        public final static Translation3d BluTeamSpeaker = new Translation3d(-8.062467,  1.442593, 2.051102);
        
    }
    
    public static final class pivotConstants {
        
        public final static double MaxSpeed = 1; //Percentage (1.0 = 100%)
        public final static double Deadzone = 0.1;
        public final static double EncoderOffset = -33;
        public final static double MinAngle = -45; //Degrees
        public final static double MaxAngle = 55;

        public final static double AmpAngle = -45;
        public final static double IntakeAngle = 60;
        public final static double SpeakerAngle = 40;
        
        public final static double kP = 0.02; // Proportion
        public final static double kI = 0.0001; // Integral
        public final static double kD = 0; // Derivative
        public final static double kTolerance = 1; // Degrees

        public final static double PivotRadius = 0.61; // Meters

    }

    public static final class climberConstants {

        // public final static double MinCycles = 0; // Minimum amount of encoder cycles (360 degrees per cycle)
        // public final static double MinDegrees = 0; // Minimum value of encoder degrees
        // public final static double MaxCycles = 0; // Maximum amount of encoder cycles (360 degrees per cycle)
        // public final static double MaxDegrees = 0; // Maximum value of encoder degrees
        // public final static double CycleTolerance = 180; // How many degrees have to change to be considered a cycle
        public final static double MinDistance = -100;
        public final static double MaxDistance = 100;

    }

}
