// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


        // Drive motor ports
        public static final int FRONT_LEFT_MOTOR_CAN = 2;
        public static final int FRONT_RIGHT_MOTOR_CAN = 4;
        public static final int BACK_LEFT_MOTOR_CAN = 1;
        public static final int BACK_RIGHT_MOTOR_CAN = 3;
    


        //Encoder constants

        public static final double ENCODER_LOW_METERS = 0.03724;
        public static final double ENCODER_LOW_METERS_PER_SECOND = 0.00062067;

        //Characterization constants
        //docs.wpilib.org "Step 2: Entering the Calculated Constants"

        //Change based on characterization
        public static final double ksVolts = 0.171;
        public static final double kvVoltSecondsPerMeter = 3.24;
        public static final double kaVoltSecondsSquaredPerMeter = 0.427;
        public static final double kPDriveVel = 2.13;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kTrackwidthMeters = 0.61;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        



        
}
