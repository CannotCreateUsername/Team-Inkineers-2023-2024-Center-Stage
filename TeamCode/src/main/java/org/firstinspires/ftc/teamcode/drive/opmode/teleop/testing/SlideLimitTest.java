package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Slide Limit Test", group = "Linear Opmode")
public class SlideLimitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor upperSlides = hardwareMap.get(DcMotor.class, "top_slide");
        DcMotor lowerSlides = hardwareMap.get(DcMotor.class, "bottom_slide");

        upperSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        upperSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                upperSlides.setPower(0.3);
                lowerSlides.setPower(0.3);
            } else {
                upperSlides.setPower(0);
                lowerSlides.setPower(0);
            }

            telemetry.addData("Encoder Position", upperSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
