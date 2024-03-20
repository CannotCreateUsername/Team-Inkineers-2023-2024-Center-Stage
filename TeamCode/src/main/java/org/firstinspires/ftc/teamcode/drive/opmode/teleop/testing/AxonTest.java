package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Axon Servo Test", group = "Linear Opmode")
public class AxonTest extends LinearOpMode {

    CRServo axon;
    AnalogInput encoder;

    int targetPos;
    final double max_servo_position = 360;
    double previous_position = 0;

    final double EXTEND_POS = 480;
    final double REST = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        axon = hardwareMap.get(CRServo.class, "axon");
        encoder = hardwareMap.get(AnalogInput.class, "axon_encoder");

        waitForStart();
        while (opModeIsActive()) {
            // get the voltage of our analog line
            // divide by 3.3 (the max voltage) to get a value between 0 and 1
            // multiply by 360 to convert it to 0 to 360 degrees
            double current_position = encoder.getVoltage() / 3.3 * max_servo_position;

            // Calculate difference (relative movement) since last reading
            double position_difference = (current_position - previous_position);

            // Wrap around logic for continuous rotation (adjust limits based on your setup)
            if (position_difference > max_servo_position / 2) {
                position_difference -= max_servo_position;
            } else if (position_difference < -max_servo_position / 2) {
                position_difference += max_servo_position;
            }

            // Calculate error based on desired change in angle and position difference
            double error = targetPos - position_difference;

            if (gamepad1.b) {
                targetPos = 0;
            } else if (gamepad1.a) {
                targetPos = 180;
            } else if (gamepad1.y) {
                targetPos = 90;
            }
            servoPID(error);

            // Update previous position for next iteration
            previous_position = current_position;

            telemetry.addData("Servo Voltage", encoder.getVoltage());
            telemetry.addData("Servo Position", current_position);
            telemetry.addData("Target Position", targetPos);
            telemetry.update();
        }
    }

    public void servoPID(double error) {
        double kP = 0.002;
        double ERR_THRESHOLD = 5;
        if (Math.abs(error) > ERR_THRESHOLD) {
            axon.setPower(Math.abs(error)*kP);
        } else {
            axon.setPower(0);
        }
    }
}
