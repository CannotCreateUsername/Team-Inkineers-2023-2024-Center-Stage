package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Axon Servo Test", group = "Linear Opmode")
public class AxonTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo axon = hardwareMap.get(Servo.class, "bar");

        axon.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b) {
                axon.setPosition(1);
            } else if (gamepad1.a) {
                axon.setPosition(0);
            }

            telemetry.addData("Servo Position", axon.getPosition());
            telemetry.update();
        }
    }
}
