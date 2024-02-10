// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Constants {

    public static final class driveConstants {

        public final static double PercentageSpeed = 0.45; //Percenage of Max Speed used for driving
        public final static double MaxSpeed = 2 * PercentageSpeed; //Max targeted speed in M/ -- 2 for now
        public final static double MaxAngularRate = 1.5 * Math.PI * PercentageSpeed; //Max targeted rotations / second -- 3/4ths of a rotation for now
        public final static double MaxAcceleration = 2 * PercentageSpeed; //Max acceleration in M/s/s;
        public final static double MaxAngularAcceleration = 4 * Math.PI * PercentageSpeed;
        public final static double SpeedDeadbandPercentage = 0.25; //Deadband or Deadzone of requested speed, as a percentage of the maximum speed;

        

        public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            new PIDConstants(0,0,0),
            new PIDConstants(0,0,0),
            0,
            0,
            new ReplanningConfig() //TODO set these properly
        );

    }

    public static final class shooterConstants {

        public final static double ShooterMaxSpeed = 0.8;
        public final static double ShooterMinSpeed = 0.4;
        public final static double ShooterBottomSpeedDifference = 0.08; //How much slower should the bottom speeds be
        public final static double AmpSpeed = 0.4; // Both motors same speed
        public final static double IntakeSpeed = 0.35;
        public final static double ReverseIntakeSpeed = -0.15;
        public final static double ReverseIntakeAutomaticSpeed = -0.1;
        public final static double ReverseDuration = 3; // Seconds
        public final static double TriggerDeadzone = 0.9;
        public final static double MotorSettledAmps = 18;

        public final static double PivotY = 0;
        
        public final static double DistanceMultiplier = 0.1;
        public final static double DistanceOffset = 0.61;

        // public final static int RedTeamSpeakerTag = 4;
        // public final static int BlueTeamSpeakerTag = 7;

        public final static Translation3d RedTeamSpeaker = new Translation3d(8.062467,  1.442593, 2.051102);
        public final static Translation3d BlueTeamSpeaker = new Translation3d(-8.062467,  1.442593, 2.051102);
        
    }
    
    public static final class pivotConstants {
        
        public final static double MaxSpeed = 1; //Percentage (1.0 = 100%)
        public final static double Deadzone = 0.1;
        public final static double EncoderOffset = 0.717;
        public final static double MinAngle = -45; //Degrees
        public final static double MaxAngle = 55;

        public final static double AmpAngle = -45;
        public final static double IntakeAngle = 55;
        public final static double SpeakerAngle = 40;
        
        public final static double kP = 0.04; // Proportion
        public final static double kI = 0.001; // Integral
        public final static double kD = 0; // Derivative
        public final static double kTolerance = 1; // Degrees

    }

}
