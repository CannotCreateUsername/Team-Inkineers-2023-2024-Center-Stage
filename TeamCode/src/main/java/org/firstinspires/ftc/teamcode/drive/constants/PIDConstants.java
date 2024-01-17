package org.firstinspires.ftc.teamcode.drive.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConstants {
    // IMU yaw constants for turning
    public static double IMUKp = 0.02;
    public static double IMUKd = 0.09;

    // Straight constants for forward/backwards
    public static double XKp = 0.05;
    public static double XKd = 0.02;

    // Lateral constants for strafing
    public static double YKp = 0.05;
    public static double YKd = 0.02;

    // ViperSlide constants
    public static double VKp = 0.003; // !! if you change higher the slides go crazy !!
    public static double VKd = 0.02;
}
