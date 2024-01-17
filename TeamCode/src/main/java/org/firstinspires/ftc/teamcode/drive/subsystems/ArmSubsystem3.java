package org.firstinspires.ftc.teamcode.drive.subsystems;



import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.VKd;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.VKp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem3 {
    public enum SlideState {
        FIRST,
        SECOND,
        THIRD,
        FOURTH,
        MANUAL,
        HANG,
        REST
    }
    public enum OuttakeState {
        OUT,
        IN,
        IDLE
    }

    private final DcMotor upperSlides;
    private final DcMotor lowerSlides;
    private final Servo virtualBar;
    private final CRServo outtake;
    private final RevTouchSensor limitSwitch;
    private final RevTouchSensor boxSwitch;

    public final int DROP = 1;
    public final int LOAD = 0;

    public double intakePower = 0.5;

    private double liftMultiplier = 1;

    // Linear Slides Positions
    private final int SLIDE_LIMIT = 1800;
    private final int firstLvl = SLIDE_LIMIT/4;
    private final int secondLvl = SLIDE_LIMIT/4*2;
    private final int thirdLvl = SLIDE_LIMIT/4*3;
    /** @noinspection FieldCanBeLocal*/
    private final int hangLvl = 500;

    SlideState slideState;
    OuttakeState outtakeState;

    private int currentTarget = 5;
    private boolean drop = false;

    private boolean hanging = false;

    ElapsedTime timer;
    ElapsedTime dropTimer;

    TriggerReader rtReader;
    TriggerReader ltReader;

    public ArmSubsystem3(HardwareMap hardwareMap) {
        // Map actuator variables to actual hardware
        upperSlides = hardwareMap.get(DcMotor.class, "top_slide");
        lowerSlides = hardwareMap.get(DcMotor.class, "bottom_slide");
        virtualBar = hardwareMap.get(Servo.class, "bar");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit_switch");
        boxSwitch = hardwareMap.get(RevTouchSensor.class, "box_switch");

        // Motor behavior setup
        upperSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        upperSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upperSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lowerSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        virtualBar.setPosition(LOAD);

        // Initialize finite state machines
        slideState = SlideState.REST;
        outtakeState = OuttakeState.IDLE;

        timer = new ElapsedTime();
        dropTimer = new ElapsedTime();
    }

    public void powerPID(double power) {
        double posErr = currentTarget - lowerSlides.getCurrentPosition(); // measure error in terms of distance between current position and target
        if (Math.abs(posErr) > 5) {
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            lowerSlides.setPower((posErr * VKp + VKd) * power); //instead of fixed power, use the concept of PID and increase power in proportion with the error
            upperSlides.setPower((posErr * VKp + VKd) * power);
        }
    }

    boolean a;

    public void runArm(GamepadEx gamepad1, GamepadEx gamepad2) {
        int positionIncrement = 50;
        double MIN_MULTIPLIER = 0.3;
        switch (slideState) {
            case REST:
                liftMultiplier = 1;
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.FIRST;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    currentTarget = 1800;
                    slideState = SlideState.HANG;
                } else if (limitSwitch.isPressed()) {
                    // Account for slippage and prevent motor stalling
                    lowerSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (timer.seconds() > 1) {
                    currentTarget = 5;
                }
                break;
            case FIRST:
                currentTarget = firstLvl;
                liftMultiplier = ((float) SLIDE_LIMIT/ lowerSlides.getCurrentPosition())/10 + MIN_MULTIPLIER; // 0.2 is the minimum multiplier
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = secondLvl;
                    slideState = SlideState.SECOND;
                } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case SECOND:
                liftMultiplier = ((float) SLIDE_LIMIT/ lowerSlides.getCurrentPosition())/10 + MIN_MULTIPLIER; // 0.2 is the minimum multiplier
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = thirdLvl;
                    slideState = SlideState.THIRD;
                } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case THIRD:
                liftMultiplier = ((float) SLIDE_LIMIT/ lowerSlides.getCurrentPosition())/10 + MIN_MULTIPLIER; // 0.2 is the minimum multiplier
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = SLIDE_LIMIT;
                    slideState = SlideState.FOURTH;
                } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    currentTarget = hangLvl;
                    slideState = SlideState.HANG;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case FOURTH:
                liftMultiplier = ((float) SLIDE_LIMIT/ lowerSlides.getCurrentPosition())/10 + MIN_MULTIPLIER; // 0.2 is the minimum multiplier
                if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    currentTarget = hangLvl;
                    slideState = SlideState.HANG;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case MANUAL:
                if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER) && currentTarget > 100) {
                    currentTarget = lowerSlides.getCurrentPosition() - positionIncrement;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = nextLvl();
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    currentTarget = hangLvl;
                    slideState = SlideState.HANG;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case HANG:
                hanging = true;
                if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    currentTarget = 0;
                    hanging = false;
                    slideState = SlideState.REST;
                }
                break;
        }
        // Code to run the virtual four bar. Don't move in REST state
        if (slideState != SlideState.REST) {
            if (gamepad1.wasJustPressed(GamepadKeys.Button.A) && !drop) {
                virtualBar.setPosition(DROP);
                drop = true;
            } else if (gamepad1.wasJustPressed(GamepadKeys.Button.A) && drop) {
                virtualBar.setPosition(LOAD);
                drop = false;
            }
        }
        if (hanging) {
            powerPID(0.6);
        } else {
            powerPID(0.6);
        }
    }

    int nextLvl() {
        int percentage = lowerSlides.getCurrentPosition()/SLIDE_LIMIT;
        if (percentage < 0.25) {
            slideState = SlideState.FIRST;
            return firstLvl;
        } else if (percentage >= 0.25 && percentage < 0.50) {
            slideState = SlideState.SECOND;
            return secondLvl;
        } else if (percentage >= 0.50 && percentage < 0.75) {
            slideState = SlideState.THIRD;
            return thirdLvl;
        } else if (percentage >= 0.75) {
            slideState = SlideState.FOURTH;
            return SLIDE_LIMIT;
        }
        return 0;
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
                    outtake.setPower(-intakePower);
                } else if ((ltReader.isDown() && !rtReader.isDown()) || boxSwitch.isPressed()) {
                    outtakeState = OuttakeState.OUT;
                    outtake.setPower(intakePower);
                }
                break;
            case IN:
                if (!rtReader.isDown()) {
                    outtakeState = OuttakeState.IDLE;
                    outtake.setPower(0);
                }
                break;
            case OUT:
                if (!ltReader.isDown() || !boxSwitch.isPressed()) {
                    outtakeState = OuttakeState.IDLE;
                    outtake.setPower(0);
                }
                break;
        }
    }

    // Telemetry
    public boolean leftBumperDown() { return a; }
    public String getLiftState() { return slideState.name(); }
    public int getSlidePosition() { return lowerSlides.getCurrentPosition(); }
    public int getCurrentTarget() { return currentTarget; }
    public double getV4bPosition() { return virtualBar.getPosition(); }
    public double getArmTimer() { return timer.seconds(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public double getSlide1Power() { return upperSlides.getPower(); }
    public double getSlide2Power() { return lowerSlides.getPower(); }

    // Autonomous Functions
    public boolean touching() { return boxSwitch.isPressed(); }
    private boolean dropped = false;

    public Action spinOuttake(double power, double duration) {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    dropTimer.reset();
                    set = true;
                }
                if (dropTimer.seconds() < duration-0.1) {
                    outtake.setPower(power);
                } else {
                    outtake.setPower(0);
                }
                return dropTimer.seconds() < duration;
            }
        };
    }

    public Action readySlides() {
        return telemetryPacket -> {
            currentTarget = 100;
            powerPID(0.4);
            return !dropped;
        };
    }

    public Action resetSlides() {
        return telemetryPacket -> {
            currentTarget = 0;
            powerPID(0.4);
            return !(lowerSlides.getCurrentPosition() <= 5);
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
                if (dropTimer.seconds() > 1.9) {
                    dropped = true;
                }
                return dropTimer.seconds() < 2;
            }
        };
    }

    public Action dropYellowPixel() {
        return new SequentialAction (
                new ParallelAction(
                        readySlides(),
                        new SequentialAction(
                                ready4bar(),
                                new SleepAction(0.5),
                                spinOuttake(0.5, 3),
                                new ParallelAction(
                                        readySlides(),
                                        reset4Bar()
                                )
                        )
                ),
                resetSlides()
        );
    }
}
