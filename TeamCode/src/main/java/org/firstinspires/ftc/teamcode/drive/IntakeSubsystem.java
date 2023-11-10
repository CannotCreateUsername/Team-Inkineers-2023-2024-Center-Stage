package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    public enum IntakeState {
        IDLE,
        IN,
        OUT
    }

    private final DcMotor intake;

    IntakeState intakeState;

    TriggerReader rtReader;
    TriggerReader ltReader;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeState = IntakeState.IDLE;
    }

    public void runIntake(GamepadEx gamepad1) {
        rtReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ltReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (intakeState) {
            case IDLE:
                if (rtReader.isDown()) {
                    intakeState = IntakeState.IN;
                    intake.setPower(1);
                } else if (ltReader.isDown() && !rtReader.isDown()) {
                    intakeState = IntakeState.OUT;
                }
                break;
            case IN:
                if (!rtReader.isDown()) {
                    intakeState = IntakeState.IDLE;
                    intake.setPower(0);
                }
                break;
            case OUT:
                intake.setPower(-1);
                if (!ltReader.isDown()) {
                    intakeState = IntakeState.IDLE;
                    intake.setPower(0);
                }
                break;
        }
    }

    public String getIntakeState() { return intakeState.name(); }

    // Autonomous Functions
}
