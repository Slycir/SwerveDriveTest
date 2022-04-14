// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class MotorControllers {
        public final static int FRONT_LEFT_DRIVE = 7;
        public final static int FRONT_RIGHT_DRIVE = 9;
        public final static int BACK_RIGHT_DRIVE = 5;
        public final static int BACK_LEFT_DRIVE = 3;

        public final static int FRONT_LEFT_STEER = 6;
        public final static int FRONT_RIGHT_STEER = 8;
        public final static int BACK_RIGHT_STEER = 4;
        public final static int BACK_LEFT_STEER = 2;

        public final static int LEAD_ENCODER = 0;
        public final static int FOLLOW_ENCODER_1 = 1;
        public final static int FOLLOW_ENCODER_2 = 2;
        public final static int FOLLOW_ENCODER_3 = 3;
    }
    public final class Ports {
        public final static int DRIVER_PORT = 0;
    }
    public final class SteerPID {
        public final static double P = 0.013;
        public final static double I = 0.0;
        public final static double D = 0.0;
    }
    public final class Etcetera {
        public final static double STEER_RATIO = 7 * 71 * (40/48);
        public final static double POWER_FACTOR = 0.5;
    }
}
