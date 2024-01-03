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

    DcMotor slides = null;
    DcMotor slides2 = null;

    ArmSubsystem arm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        while (!selected) {
            if (gamepad1.x) {
                hardcode = true;
                slides = hardwareMap.get(DcMotor.class, "slides");
                slides2 = hardwareMap.get(DcMotor.class, "slides2");
                slides.setDirection(DcMotorSimple.Direction.REVERSE);
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides2.setDirection(DcMotorSimple.Direction.REVERSE);
                slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                selected = true;
            } else if (gamepad1.y) {
                hardcode = false;
                arm = new ArmSubsystem(hardwareMap);
                selected = true;
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
            if (gamepad1.dpad_down && hardcode) {
                slides.setPower(-1);
                slides2.setPower(-1);
            } else if (gamepad1.dpad_up && hardcode) {
                slides.setPower(0.5);
                slides2.setPower(0.5);
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
}
