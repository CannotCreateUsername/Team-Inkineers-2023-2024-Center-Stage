package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;

@TeleOp(name = "Wait Test", group = "Linear Opmode")
public class WaitForClearTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ComputerVisionMediator CV = new ComputerVisionMediator();
        CV.initLight(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Actions.runBlocking(CV.waitForClear());
            }
        }
    }
}
