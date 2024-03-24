package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Axon Servo Testi", group = "Linear Opmode")
public class AxonTest2 extends LinearOpMode {

    CRServo axon = null;
    AnalogInput encoder = null;

    final double max_servo_position = 360;
    final double rotation_threshold = 1;

    static double absPos = 0;
    double position = 0;

    double targetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        axon = hardwareMap.get(CRServo.class, "bar_left");
        encoder = hardwareMap.get(AnalogInput.class, "left_axon_encoder");

        axon.setDirection(DcMotorSimple.Direction.REVERSE);

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        absPos = 0;
        while (opModeInInit()) {
            axon.setPower(0.01);
            position = encoder.getVoltage() / 3.3 * max_servo_position;
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Absolute Position", absPos);
            telemetry.update();
        }

        // Positive target for CLOCKWISE, Negative for COUNTERCLOCKWISE
        waitForStart();
        while (opModeIsActive()) {
            updatePos();

            if (gamepad1.a) {
                targetPos = 0;
            } else if (gamepad1.b) {
                targetPos = -90;
            } else if (gamepad1.y) {
                targetPos = 360;
            } else if (gamepad1.x) {
                targetPos = 90;
            } else if (gamepad1.dpad_up) {
                targetPos = 480;
            } else if (gamepad1.dpad_down) {
                targetPos = -480;
            }

            if (gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                targetPos += 10;
            } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                targetPos -= 10;
            }

            double error = targetPos - absPos;
            servoPID(error);
            gamepadEx.readButtons();

            // Output values for troubleshooting
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Absolute Position", absPos);
            telemetry.addData("Error", error);
            telemetry.update();
        }

    }

    private void updatePos() {
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        double currentPos = encoder.getVoltage() / 3.3 * max_servo_position;
        double previousPos = position;

        // Calculate clockwise (CW) and counterclockwise (CCW) distances
        double CWDistance = (currentPos - previousPos + 360) % 360;
        double CCWDistance = (previousPos - currentPos + 360) % 360;

        position = currentPos;

        // Update absolute position
        if (Math.abs(CWDistance - CCWDistance) < rotation_threshold) {
            // If the difference between CW and CCW distances is small, handle transition across 360 degrees
            if (currentPos < previousPos) {
                absPos += CWDistance;
            } else {
                absPos -= CCWDistance;
            }
        } else {
            // Choose the direction with the smaller distance
            if (CWDistance < CCWDistance) {
                absPos += CWDistance;
            } else {
                absPos -= CCWDistance;
            }
        }
    }

    public void servoPID(double error) {
        double kP = 0.005;
        double kD = 0.08; //0.08
        double ERR_THRESHOLD = 1;
        if (Math.abs(error) > ERR_THRESHOLD) {
            // Ensure the servo rotates in the correct direction based on the error sign
            axon.setPower(error > 0 ? Math.abs(error) * kP + kD : -Math.abs(error) * kP);
        } else {
            axon.setPower(0); // Stop the servo if the error is within the threshold
        }
    }
}
