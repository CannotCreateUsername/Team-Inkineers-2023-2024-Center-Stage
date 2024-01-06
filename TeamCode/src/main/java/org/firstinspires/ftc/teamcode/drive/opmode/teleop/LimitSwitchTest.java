package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "JAJA tester", group = "Linear Opmode")
public class LimitSwitchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevTouchSensor boxSwitch = hardwareMap.get(RevTouchSensor.class, "box_switch");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Sensor pressed?", boxSwitch.isPressed());
            telemetry.update();
        }
    }
}
