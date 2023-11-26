package org.firstinspires.ftc.teamcode.drive.subsystems;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        REST,
        HANG
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

    public final int DROP = -1;
    public final int LOAD = 1;

    private double liftMultiplier = 1;

    SlideState slideState;
    OuttakeState outtakeState;

    private double currentTarget;
    private boolean reversed = false;
    private boolean dropped = false;

    ElapsedTime timer;
    ElapsedTime dropTimer;

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

        virtualBar.setPosition(LOAD);

        // Initialize finite state machines
        slideState = SlideState.REST;
        outtakeState = OuttakeState.IDLE;

        timer = new ElapsedTime();
        dropTimer = new ElapsedTime();
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

    boolean a;

    public void runArm(GamepadEx gamepad1, GamepadEx gamepad2) {
        int SLIDE_LIMIT = 1800;
        double MIN_MULTIPLIER = 0.3;
        switch (slideState) {
            case REST:
                liftMultiplier = 1;
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.READY;
                    timer.reset();
                } else if (gamepad2.wasJustReleased(GamepadKeys.Button.A)) {
                    slideState = SlideState.HANG;
                }
                if (timer.seconds() > 1.5) {
                    runToPosition(50);
                }
                break;
            case READY:
                liftMultiplier = 1;
                runToPosition(300, 0.2);
                if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && timer.seconds() > 0.5) {
                    slideState = SlideState.RUNNING;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                } else if (timer.seconds() > 0.5) {
                    virtualBar.setPosition(DROP);
                }
                break;
            case RUNNING:
                liftMultiplier = ((float) SLIDE_LIMIT/slides.getCurrentPosition())/10 + MIN_MULTIPLIER; // 0.2 is the minimum multiplier
                if ((gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER) || gamepad2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) && slides.getCurrentPosition() < SLIDE_LIMIT) {
                    if (reversed) {
                        runToPosition(slides.getCurrentPosition()-100);
                    } else {
                        runToPosition(slides.getCurrentPosition()+100);
                    }
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.PAUSED;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case PAUSED:
                liftMultiplier = ((float) SLIDE_LIMIT/slides.getCurrentPosition())/10 + MIN_MULTIPLIER;
                if (gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.RUNNING;
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case HANG:
                liftMultiplier = 0.8;
                if (gamepad2.wasJustReleased(GamepadKeys.Button.A)) {
                    slideState = SlideState.REST;
                } else if (gamepad2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    reversed = true;
                    slideState = SlideState.RUNNING;
                }
                runToPosition(1000, 0.2);
                break;
        }
        if (gamepad1.wasJustPressed(GamepadKeys.Button.X) || gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
            reversed = !reversed;
        }
        a = gamepad2.isDown(GamepadKeys.Button.RIGHT_BUMPER);
    }

    // Decrease driving speed for more control when the slides are lifted
    public double getPowerMultiplier() { return liftMultiplier; }

    public void runOuttake(GamepadEx gamepad1) {
        rtReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ltReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (outtakeState) {
            case IDLE:
                if (rtReader.isDown()) {
                    outtakeState = OuttakeState.IN;
                    outtake.setPower(-1);
                } else if (ltReader.isDown() && !rtReader.isDown()) {
                    outtakeState = OuttakeState.OUT;
                }
                break;
            case IN:
                if (!rtReader.isDown()) {
                    outtakeState = OuttakeState.IDLE;
                    outtake.setPower(0);
                }
                break;
            case OUT:
                outtake.setPower(1);
                if (!ltReader.isDown()) {
                    outtakeState = OuttakeState.IDLE;
                    outtake.setPower(0);
                }
                break;
        }
    }

    public void runToPosition(int position, double power) {
        currentTarget = position;
        slides.setTargetPosition(position);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(powerPID(power));
    }

    // Run to position at default power
    public void runToPosition(int position) {
        runToPosition(position, 0.4);
    }

    // Telemetry
    public boolean rightBumperDown() { return a; }
    public String getLiftState() { return slideState.name(); }
    public int getSlidePosition() { return slides.getCurrentPosition(); }
    public double getV4bPosition() { return virtualBar.getPosition(); }
    public double getArmTimer() { return timer.seconds(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public boolean isReversed() { return reversed; }

    // Autonomous Functions

    public Action spinOuttake(double power) {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    dropTimer.reset();
                    set = true;
                }
                if (dropTimer.seconds() < 1.9) {
                    outtake.setPower(power);
                } else {
                    outtake.setPower(0);
                    dropped = true;
                }
                return dropTimer.seconds() < 2;
            }
        };
    }

    public Action readySlides() {
        return telemetryPacket -> {
            runToPosition(200, 0.2);
            return !dropped;
        };
    }

    public Action resetSlides() {
        return telemetryPacket -> {
            runToPosition(50);
            return !(slides.getCurrentPosition() <= 50);
        };
    }

    public Action ready4bar() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    dropTimer.reset();
                    set = true;
                }
                virtualBar.setPosition(DROP);
                return dropTimer.seconds() < 2;
            }
        };
    }

    public Action reset4Bar() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    dropTimer.reset();
                    set = true;
                }
                virtualBar.setPosition(LOAD);
                return dropTimer.seconds() < 2;
            }
        };
    }

    public Action dropYellowPixel() {
        return new SequentialAction(
                new ParallelAction(
                        readySlides(),
                        ready4bar(),
                        new SequentialAction(
                                new SleepAction(3),
                                spinOuttake(1))
                ),
                reset4Bar(),
                resetSlides()
        );
    }
}
