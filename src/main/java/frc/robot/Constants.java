// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int LEFT_MOTOR_PIN_1 = 6;
        public static final int LEFT_MOTOR_PIN_2 = 5;
        public static final int LEFT_MOTOR_PIN_3 = 7;
    
        public static final int RIGHT_MOTOR_PIN_1 = 1;
        public static final int RIGHT_MOTOR_PIN_2 = 31;
        public static final int RIGHT_MOTOR_PIN_3 = 3;
    
        public static final double GYRO_KP = 0.0018;
        public static final double GYRO_KI = 0;
        public static final double GYRO_KD = 0.1;

        public static final String USB_CAMERA_NAME = "HD_Webcam_C525";

      }
    
}