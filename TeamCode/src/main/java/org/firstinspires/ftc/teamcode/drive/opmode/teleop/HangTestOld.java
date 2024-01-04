package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;

//@Disabled
@TeleOp(name = "Hang Weirder", group = "Linear Opmode")
public class HangTestOld extends LinearOpMode {
    boolean selected = false;
    boolean hardcode = false;
    boolean selected2 = false;

    DcMotor slides = null;
    DcMotor slides2 = null;

    ArmSubsystem arm = null;

    private double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        while (!selected) {
            if (gamepad1.x) {
                hardcode = true;
                slides = hardwareMap.get(DcMotor.class, "slides");
                slides2 = hardwareMap.get(DcMotor.class, "slides2");
                slides.setDirection(DcMotorSimple.Direction.REVERSE);
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides2.setDirection(DcMotorSimple.Direction.REVERSE);
                slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                selected = true;
            } else if (gamepad1.y) {
                hardcode = false;
                arm = new ArmSubsystem(hardwareMap);
                selected = true;
            }

            if (hardcode) {
                while (!selected2) {
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
                slides.setPower(-1);
                slides2.setPower(-1);
            } else if (gamepad1.dpad_up && hardcode && !selected2) {
                slides.setPower(0.5);
                slides2.setPower(0.5);
            } else if (hardcode && selected2) {
                if (gamepad1.dpad_up && position <= 1900) {
                    position += 100;
                } else if (gamepad1.dpad_down && position > 100) {
                    position -= 100;
                }
                PowerP();
            } else {
                if (hardcode) {
                    slides.setPower(0);
                    slides2.setPower(0);
                }
            }

            if (!hardcode) {
                arm.runArm(gamepadEx1, gamepadEx2);
            }
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();
        }
    }

    private void PowerP() {
        double kP = 0.05;
        double error = position - slides.getCurrentPosition();
        if (Math.abs(error) > 2) {
            slides.setPower(error * kP);
            slides2.setPower(error * kP);
        }
    }
}
