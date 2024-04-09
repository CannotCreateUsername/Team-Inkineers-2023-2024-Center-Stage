package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class V4BSubsystem2 {

    public final Servo rightVirtualBar;
    public final Servo leftVirtualBar;

    // Right Servo Positions
    public final double rLOAD = 1;
    public final double rDROP = 0;
    public final double rHANG = 1-0.6; // 220/360
    public final double rHANG2 = 1-0.3;

    // Left Servo Positions
    public final double lLOAD = 0;
    public final double lDROP = 1;
    public final double lHANG = 0.6;
    public final double lHANG2 = 0.3;

    // Constructor
    public V4BSubsystem2(HardwareMap hardwareMap) {
        rightVirtualBar = hardwareMap.get(Servo.class, "bar_right");
        leftVirtualBar = hardwareMap.get(Servo.class, "bar_left");
    }

    // Functions
    public void extend() {
        rightVirtualBar.setPosition(rDROP);
        leftVirtualBar.setPosition(lDROP);
    }
    public void retract() {
        rightVirtualBar.setPosition(rLOAD);
        leftVirtualBar.setPosition(lLOAD);
    }
    public void hang() {
        rightVirtualBar.setPosition(rHANG);
        leftVirtualBar.setPosition(lHANG);
    }
    public void hangRest() {
        rightVirtualBar.setPosition(rHANG2);
        leftVirtualBar.setPosition(lHANG2);
    }
}
