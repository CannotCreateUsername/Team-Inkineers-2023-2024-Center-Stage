package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Axon Position Test", group = "Linear Opmode")
public class AxonTestPosition extends LinearOpMode {
    final double rLoad = 0;
    final double lLoad = 0;
    final double rRest = 0;
    final double lRest = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo right = hardwareMap.get(Servo.class, "right_bar");
        Servo left = hardwareMap.get(Servo.class, "left_bar");

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
