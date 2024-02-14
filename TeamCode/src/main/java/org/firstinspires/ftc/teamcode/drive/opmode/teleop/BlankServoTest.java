package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Dual Servo Test", group = "Linear Opmode")
public class BlankServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo right = hardwareMap.get(Servo.class, "bar_right");
        Servo left = hardwareMap.get(Servo.class, "bar_left");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                right.setPosition(1-0.296);
                left.setPosition(0.296);
            } else if (gamepad1.b) {
                right.setPosition(1);
                left.setPosition(0);
            }
        }
    }
}
