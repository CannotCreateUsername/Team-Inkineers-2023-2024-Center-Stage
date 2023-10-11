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
    public enum IntakeBarState {
        FOLDED,
        UNFOLDED
    }
    public enum IntakeState {
        IDLE,
        IN,
        OUT
    }

    private final static double Kp = .05;

    private DcMotor intakeBar = null;
    private CRServo intakeWheel = null;

    IntakeBarState intakeBarState;
    IntakeState intakeState;


    double currentTarget;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap) {
        intakeBar = hardwareMap.get(DcMotor.class, "intakeBar");
        intakeBar.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeWheel = hardwareMap.get(CRServo.class, "intakeWheel");
        intakeBarState = IntakeBarState.FOLDED;
        intakeState = IntakeState.IDLE;
    }

    public double powerPID(double power) {
        if (Math.abs(intakeBar.getCurrentPosition() - currentTarget) > 15){
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            return power;
        } else {
            double posErr = currentTarget - intakeBar.getCurrentPosition(); // measure error in terms of distance between current position and target
            return (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
        }
    }

    public void runIntake(GamepadEx gamepad1) {
        TriggerReader rtReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        TriggerReader ltReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (intakeState) {
            case IDLE:
                intakeWheel.setPower(0);
                if (rtReader.isDown()) {
                    intakeState = IntakeState.IN;
                } else if (ltReader.isDown() && !rtReader.isDown()) {
                    intakeState = IntakeState.OUT;
                }
            case IN:
                intakeWheel.setPower(1);
                if (rtReader.wasJustReleased()) {
                    intakeState = IntakeState.IDLE;
                }
            case OUT:
                intakeWheel.setPower(-1);
                if (ltReader.wasJustReleased()) {
                    intakeState = IntakeState.IDLE;
                }
        }
    }

    public void runIntakeLift(GamepadEx gamepad1) {
        switch (intakeBarState) {
            case FOLDED:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                    runToPosition(-200);
                    intakeBarState = IntakeBarState.UNFOLDED;
                }
            case UNFOLDED:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
                    runToPosition(0);
                    intakeBarState = IntakeBarState.FOLDED;
                }
        }
    }

    public void runToPosition(int position, double power) {
        currentTarget = position;
        intakeBar.setTargetPosition(position);
        intakeBar.setPower(powerPID(power));
    }

    // Run to position at default power
    public void runToPosition(int position) {
        runToPosition(position, 0.5);
    }

    public String getIntakeState() { return intakeBarState.name() + intakeState.name(); }
}
