package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Hang Weird", group = "Linear Opmode")
public class HangTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                slides.setPower(-1);
            } else {
                slides.setPower(0);
            }
        }
    }
}
