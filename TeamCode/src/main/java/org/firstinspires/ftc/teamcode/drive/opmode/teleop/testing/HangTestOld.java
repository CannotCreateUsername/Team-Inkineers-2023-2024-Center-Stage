package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem4;

@Disabled
@TeleOp(name = "Hang Weirder", group = "Linear Opmode")
public class HangTestOld extends LinearOpMode {
    boolean selected = false;
    boolean hardcode = false;
    boolean selected2 = false;

    DcMotor upperSlides = null;
    DcMotor lowerSlides = null;

    ArmSubsystem4 arm = null;

    private double currentTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        while (!selected && !isStopRequested()) {
            if (gamepad1.x) {
                hardcode = true;
                upperSlides = hardwareMap.get(DcMotor.class, "top_slide");
                lowerSlides = hardwareMap.get(DcMotor.class, "bottom_slide");
                upperSlides.setDirection(DcMotorSimple.Direction.REVERSE);
                upperSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                upperSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lowerSlides.setDirection(DcMotorSimple.Direction.REVERSE);
                lowerSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                selected = true;
            } else if (gamepad1.y) {
                hardcode = false;
                arm = new ArmSubsystem4(hardwareMap, true);
                selected = true;
            }

            if (hardcode) {
                while (!selected2 && !isStopRequested()) {
                    if (gamepad1.a) {
                        break;
                    } else if (gamepad1.b) {
                        selected2 = true;
                    }
                    telemetry.addData("Select mode again", "A for normal, B for PID");
                    telemetry.update();
                }
            }

            telemetry.addData("Select your mode", "X for simple, Y for subsystem");
            telemetry.update();
        }

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        telemetry.addData("Ready", "Press play to start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_down && hardcode && !selected2) {
                upperSlides.setPower(-0.5);
                lowerSlides.setPower(-0.5);
            } else if (gamepad1.dpad_up && hardcode && !selected2) {
                upperSlides.setPower(0.5);
                lowerSlides.setPower(0.5);
            } else if (hardcode && selected2) {
                if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER) && currentTarget <= 1900) {
                    currentTarget += 25;
                } else if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER) && currentTarget > 25) {
                    currentTarget -= 25;
                }
                PowerP();
            } else {
                if (hardcode && gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_UP) && !selected2) {
                    upperSlides.setPower(0);
                    lowerSlides.setPower(0);
                }
            }

            if (!hardcode) {
                arm.runArm(gamepadEx1, gamepadEx2);
                telemetry.addData("Encoder Position", arm.getSlidePosition());
                telemetry.addData("is left bumper down", arm.leftBumperDown());
            } else {
                telemetry.addData("Encoder Position", lowerSlides.getCurrentPosition());
            }
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();
            telemetry.update();
        }
    }

    private void PowerP() {
        double kP = 0.01;
        double error = currentTarget - lowerSlides.getCurrentPosition();
        if (Math.abs(error) > 2) {
            upperSlides.setPower(error * kP * 0.6);
            lowerSlides.setPower(error * kP * 0.6);
        }
    }
}
