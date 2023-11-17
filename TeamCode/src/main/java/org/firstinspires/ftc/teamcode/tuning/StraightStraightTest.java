package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "straight yesy", group = "Linear Opmode")
public class StraightStraightTest extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontRight;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                timer.reset();
                while (timer.seconds() < 2) {
                    frontLeft.setPower(0.4);
                    backLeft.setPower(0.4);
                    backRight.setPower(0.4);
                    frontRight.setPower(0.4);
                }
            }
            if (gamepad1.b) {
                timer.reset();
                while (timer.seconds() < 2) {
                    frontLeft.setPower(-0.4);
                    backLeft.setPower(-0.4);
                    backRight.setPower(-0.4);
                    frontRight.setPower(-0.4);
                }
            }
        }
    }
}
