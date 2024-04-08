package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.V4BSubsystem;

@TeleOp(name = "Dual Axon Test", group = "Linear Opmode")
public class DualAxonTest extends LinearOpMode {

    V4BSubsystem v4B;

    @Override
    public void runOpMode() throws InterruptedException {
        v4B = new V4BSubsystem(hardwareMap);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        while (opModeInInit()) {
            v4B.init(this);
            telemetry.update();
        }

        // Positive target for CLOCKWISE, Negative for COUNTERCLOCKWISE
        waitForStart();
        while (opModeIsActive()) {
            v4B.updatePosAll();

            if (gamepadEx.isDown(GamepadKeys.Button.BACK)) {
                v4B.resetToRest();
            } else {
                if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
                    v4B.extend();
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
                    v4B.retract();
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
                    v4B.hang();
                }
            }
            gamepadEx.readButtons();

            v4B.servoPID();

            // Output values for troubleshooting
            telemetry.addData("Target Position", v4B.getTargetPos());
            telemetry.addData("RIGHT Absolute Position", v4B.getRightAbsolutePosition());
            telemetry.addData("LEFT Absolute Position", v4B.getLeftAbsolutePosition());
            telemetry.addData("Right Axon Error", v4B.getRightError());
            telemetry.addData("Left Axon Error", v4B.getLeftError());
            telemetry.addLine("X: Extend, A: Retract, B: Hang");
            telemetry.update();
        }

    }
}
