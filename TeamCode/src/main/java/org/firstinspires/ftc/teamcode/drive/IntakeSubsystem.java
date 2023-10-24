package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    public enum IntakeState {
        IDLE,
        IN,
        OUT
    }

    private final static double Kp = .05;

    private DcMotor intake = null;

    IntakeState intakeState;


    double currentTarget;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeState = IntakeState.IDLE;
    }

//    public double powerPID(double power) {
//        if (Math.abs(intakeBar.getCurrentPosition() - currentTarget) > 15){
//            // our threshold is within
//            // 15 encoder ticks of our target.
//            // this is pretty arbitrary, and would have to be
//            // tweaked for each robot.
//            return power;
//        } else {
//            double posErr = currentTarget - intakeBar.getCurrentPosition(); // measure error in terms of distance between current position and target
//            return (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
//        }
//    }

    public void runIntake(GamepadEx gamepad1) {
        TriggerReader rtReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        TriggerReader ltReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (intakeState) {
            case IDLE:
                intake.setPower(0);
                if (rtReader.isDown()) {
                    intakeState = IntakeState.IN;
                } else if (ltReader.isDown() && !rtReader.isDown()) {
                    intakeState = IntakeState.OUT;
                }
            case IN:
                intake.setPower(1);
                if (rtReader.wasJustReleased()) {
                    intakeState = IntakeState.IDLE;
                }
            case OUT:
                intake.setPower(-1);
                if (ltReader.wasJustReleased()) {
                    intakeState = IntakeState.IDLE;
                }
        }
    }

//    public void runToPosition(int position, double power) {
//        currentTarget = position;
//        intakeBar.setTargetPosition(position);
//        intakeBar.setPower(powerPID(power));
//    }
//
//    // Run to position at default power
//    public void runToPosition(int position) {
//        runToPosition(position, 0.5);
//    }

    public String getIntakeState() { return intakeState.name(); }
}
