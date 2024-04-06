package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class V4BSubsystem {

    private final CRServo rightVirtualBar;
    private final CRServo leftVirtualBar;
    private final AnalogInput rightEncoder;
    private final AnalogInput leftEncoder;

    // VIRTUAL FOUR BAR LOOP STUFF
    final double max_servo_position = 360;
    final double rotation_threshold = 1;

    double right_absolute_position;
    double left_absolute_position;
    double right_position = 0;
    double left_position = 0;

    private double targetPos = 0;
    private double rightError = 0;
    private double leftError = 0;

    // Constructor
    public V4BSubsystem(HardwareMap hardwareMap) {
        rightVirtualBar = hardwareMap.get(CRServo.class, "bar_right");
        leftVirtualBar = hardwareMap.get(CRServo.class, "bar_left");
        rightEncoder = hardwareMap.get(AnalogInput.class, "right_axon_encoder");
        leftEncoder = hardwareMap.get(AnalogInput.class, "left_axon_encoder");

        // Reverse V4B servos
        rightVirtualBar.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVirtualBar.setDirection(DcMotorSimple.Direction.REVERSE);

        right_absolute_position = 0;
        left_absolute_position = 0;
    }

    public void init(LinearOpMode opMode) {
        rightVirtualBar.setPower(0.01);
        leftVirtualBar.setPower(0.01);
        right_position = rightEncoder.getVoltage() / 3.3 * max_servo_position;
        left_position = leftEncoder.getVoltage() / 3.3 * max_servo_position;
        opMode.telemetry.addData("Target Position", getTargetPos());
        opMode.telemetry.addData("RIGHT Absolute Position", getRightAbsolutePosition());
        opMode.telemetry.addData("LEFT Absolute Position", getLeftAbsolutePosition());
        opMode.telemetry.addData("Right Axon Error", getRightError());
        opMode.telemetry.addData("Left Axon Error", getLeftError());
    }

    // Positive target for CLOCKWISE, Negative for COUNTERCLOCKWISE, target in DEGREES
    public void extend() {
        targetPos = -360;
    }
    public void retract() {
        targetPos = 0;
    }
    public void hang() { targetPos = -200; }
    public void setTarget(double target) {
        targetPos = target;
    }
    public void resetToRest() {
        rightVirtualBar.setPower(-0.4);
        leftVirtualBar.setPower(0.4);
        right_absolute_position = 0;
//        right_position = 0;
        left_absolute_position = 0;
//        left_position = 0;
    }

    public void servoPID() {
        double leftTargetPos = -targetPos;
        rightError = targetPos - right_absolute_position;
        leftError = leftTargetPos - left_absolute_position;
        double kP = 0.005;
        double kD_R = 0.08; //0.08
        double kD_L = 0.08;
        double ERR_THRESHOLD = 10;
        if (Math.abs(rightError) > ERR_THRESHOLD || Math.abs(leftError) > ERR_THRESHOLD) {
            // Ensure the servo rotates in the correct direction based on the error sign
            rightVirtualBar.setPower(rightError > 0 ? Math.abs(rightError) * kP + kD_R : -Math.abs(rightError) * kP);
            leftVirtualBar.setPower(leftError > 0 ? Math.abs(leftError) * kP : -Math.abs(leftError) * kP);
        } else {
            rightVirtualBar.setPower(0); // Stop the servo if the error is within the threshold
            leftVirtualBar.setPower(0);
        }
    }

    // TELEMETRY
    public double getRightAbsolutePosition() { return right_absolute_position; }
//    public double getRightPosition() { return right_position; }
    public double getLeftAbsolutePosition() { return left_absolute_position; }
//    public double getLeftPosition() { return left_position; }
    public double getRightError() { return rightError; }
    public double getLeftError() { return leftError; }
    public double getTargetPos() { return targetPos; }

    public void updatePosAll() {
        updatePosRight();
        updatePosLeft();
    }

    // for RIGHT servo
    public void updatePosRight() {
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        double currentPos = rightEncoder.getVoltage() / 3.3 * max_servo_position;
        double previousPos = right_position;

        // Calculate clockwise (CW) and counterclockwise (CCW) distances
        double CWDistance = (currentPos - previousPos + 360) % 360;
        double CCWDistance = (previousPos - currentPos + 360) % 360;

        right_position = currentPos;

        // Update absolute position
        if (Math.abs(CWDistance - CCWDistance) < rotation_threshold) {
            // If the difference between CW and CCW distances is small, handle transition across 360 degrees
            if (currentPos < previousPos) {
                right_absolute_position += CWDistance;
            } else {
                right_absolute_position -= CCWDistance;
            }
        } else {
            // Choose the direction with the smaller distance
            if (CWDistance < CCWDistance) {
                right_absolute_position += CWDistance;
            } else {
                right_absolute_position -= CCWDistance;
            }
        }
    }
    // for LEFT servo
    public void updatePosLeft() {
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        double currentPos = leftEncoder.getVoltage() / 3.3 * max_servo_position;
        double previousPos = left_position;

        // Calculate clockwise (CW) and counterclockwise (CCW) distances
        double CWDistance = (currentPos - previousPos + 360) % 360;
        double CCWDistance = (previousPos - currentPos + 360) % 360;

        left_position = currentPos;

        // Update absolute position
        if (Math.abs(CWDistance - CCWDistance) < rotation_threshold) {
            // If the difference between CW and CCW distances is small, handle transition across 360 degrees
            if (currentPos < previousPos) {
                left_absolute_position += CWDistance;
            } else {
                left_absolute_position -= CCWDistance;
            }
        } else {
            // Choose the direction with the smaller distance
            if (CWDistance < CCWDistance) {
                left_absolute_position += CWDistance;
            } else {
                left_absolute_position -= CCWDistance;
            }
        }
    }
}
