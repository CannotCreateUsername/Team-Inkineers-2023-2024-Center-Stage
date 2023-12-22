package org.firstinspires.ftc.teamcode.drive.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    ElapsedTime intakeTimer;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeState = IntakeState.IDLE;
    }

    public void runIntake(GamepadEx gamepad1) {
        rtReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ltReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (intakeState) {
            case IDLE:
                if (rtReader.isDown()) {
                    intakeState = IntakeState.IN;
                    intake.setPower(0.5);
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
                intake.setPower(-0.5);
                if (!ltReader.isDown()) {
                    intakeState = IntakeState.IDLE;
                    intake.setPower(0);
                }
                break;
        }
    }

    public String getIntakeState() { return intakeState.name(); }

    // Autonomous Functions
    public Action spinIntake(double power, double duration) {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    intakeTimer.reset();
                    set = true;
                }
                if (intakeTimer.seconds() < duration-0.1) {
                    intake.setPower(power);
                } else {
                    intake.setPower(0);
                }
                return intakeTimer.seconds() < duration;
            }
        };
    }
}
