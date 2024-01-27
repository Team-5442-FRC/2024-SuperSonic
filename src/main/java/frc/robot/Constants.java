// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

        public final static double MaxSpeed = 0.2; //Percentage (1.0 = 100%)
        public final static double ShooterTopSpeed = 1;
        public final static double ShooterBottomSpeed = 0.92;
        public final static double IntakeSpeed = 0.35;
        public final static double ReverseIntakeSpeed = -0.15;
        public final static double ReverseIntakeAutomaticSpeed = -0.15;
        public final static double ReverseDuration = 3; // Seconds
        public final static double TriggerDeadzone = 0.9;
        public final static double MotorSettledAmps = 18;

    }

}
