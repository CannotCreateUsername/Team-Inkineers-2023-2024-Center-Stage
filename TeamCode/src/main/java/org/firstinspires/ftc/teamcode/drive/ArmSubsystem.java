package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem {
    public enum SlideState {
        READY,
        RUNNING,
        PAUSED,
        REST
    }
    public enum OuttakeState {
        OUT,
        IN,
        IDLE
    }

    private final static double Kp = .05;

    private final DcMotor slides;
    private final Servo virtualBar;
    private final CRServo outtake;

    public final int DROP = 1;
    public final int LOAD = 0;

    SlideState slideState;
    OuttakeState outtakeState;

    double currentTarget;

    ElapsedTime timer;

    TriggerReader rtReader;
    TriggerReader ltReader;

    public ArmSubsystem(HardwareMap hardwareMap) {
        // Map actuator variables to actual hardware
        slides = hardwareMap.get(DcMotor.class, "slides");
        virtualBar = hardwareMap.get(Servo.class, "bar");
        outtake = hardwareMap.get(CRServo.class, "outtake");

        // Motor behavior setup
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize finite state machines
        slideState = SlideState.REST;
        outtakeState = OuttakeState.IDLE;

        timer = new ElapsedTime();
    }

    public double powerPID(double power) {
        if (Math.abs(slides.getCurrentPosition() - currentTarget) > 15){
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            return power;
        } else {
            double posErr = currentTarget - slides.getCurrentPosition(); // measure error in terms of distance between current position and target
            return (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
        }
    }

    public void runArm(GamepadEx gamepad1) {
        switch (slideState) {
            case REST:
                if (gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    runToPosition(100);
                    slideState = SlideState.READY;
                    timer.reset();
                }
                break;
            case READY:
                if (gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.RUNNING;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    runToPosition(0);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                if (timer.seconds() > 1) {
                    virtualBar.setPosition(DROP);
                }
                break;
            case RUNNING:
                runToPosition(slides.getCurrentPosition()+2);
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.PAUSED;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    runToPosition(0);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case PAUSED:
                runToPosition(slides.getCurrentPosition());
                if (gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.RUNNING;
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    runToPosition(0);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
        }
    }

    public void runOuttake(GamepadEx gamepad2) {
        rtReader = new TriggerReader(gamepad2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ltReader = new TriggerReader(gamepad2, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (outtakeState) {
            case IDLE:
                outtake.setPower(0);
                if (ltReader.isDown()) {
                    outtakeState = OuttakeState.OUT;
                } else if (rtReader.isDown()) {
                    outtakeState = OuttakeState.IN;
                }
                break;
            case IN:
                outtake.setPower(1);
                if (ltReader.wasJustReleased()) {
                    outtakeState = OuttakeState.IDLE;
                }
                break;
            case OUT:
                outtake.setPower(-1);
                if (rtReader.wasJustReleased()) {
                    outtakeState = OuttakeState.IDLE;
                }
                break;
        }
    }

    public void runToPosition(int position, double power) {
        currentTarget = position;
        slides.setTargetPosition(position);
        slides.setPower(powerPID(power));
    }

    // Run to position at default power
    public void runToPosition(int position) {
        runToPosition(position, 0.5);
    }

    // Telemetry
    public String getLiftState() { return slideState.name(); }

    // Autonomous Functions
    public Action dropYellowPixel() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                runToPosition(200);
                virtualBar.setPosition(DROP);
                return false;
            }
        }
    }
}
