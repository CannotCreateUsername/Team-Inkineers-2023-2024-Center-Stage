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
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import javax.annotation.Nullable;


public class ArmSubsystem4 {
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

    // ViperSlide PID constants
    private double VKp = 0.0015; // !! if you change higher the slides go crazy !!

    V4BSubsystem v4B;

    private final DcMotor upperSlides;
    private final DcMotor lowerSlides;
    private final CRServo outtake;
    private final RevTouchSensor limitSwitch;
    private final RevTouchSensor boxSwitch;

    public double intakePower = 0.5;
    public double hangingMultiplier = 0.8;

    private double liftMultiplier = 1;

    // Linear Slides Positions
    private final int SLIDE_LIMIT = 1800;
    private final int firstLvl = SLIDE_LIMIT/4-20;
    private final int secondLvl = SLIDE_LIMIT/4*2;
    private final int thirdLvl = SLIDE_LIMIT/4*3;
    /** @noinspection FieldCanBeLocal*/

    SlideState slideState;
    OuttakeState outtakeState;

    private int currentTarget = 5;
    private boolean drop = false;

    private boolean hanging = false;
    private boolean hangRelease = false;

    ElapsedTime timer;
    ElapsedTime dropTimer;
    ElapsedTime barTimer;
    ElapsedTime hangTimer;
    ElapsedTime slideTimer;

    TriggerReader rtReader;
    TriggerReader ltReader;

    LinearOpMode opMode;

    public ArmSubsystem4(HardwareMap hardwareMap, boolean auto) {
        this(hardwareMap, auto,null);
    }

    public ArmSubsystem4(HardwareMap hardwareMap, boolean auto, @Nullable LinearOpMode linearOpMode) {
        opMode = linearOpMode;
        // Initialize the virtual four bar
        v4B = new V4BSubsystem(hardwareMap, auto);
        // Map actuator variables to actual hardware
        upperSlides = hardwareMap.get(DcMotor.class, "top_slide");
        lowerSlides = hardwareMap.get(DcMotor.class, "bottom_slide");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit_switch");
        boxSwitch = hardwareMap.get(RevTouchSensor.class, "box_switch");

        // Motor behavior setup
        upperSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        upperSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upperSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        lowerSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lowerSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize finite state machines
        slideState = SlideState.REST;
        outtakeState = OuttakeState.IDLE;

        timer = new ElapsedTime();
        dropTimer = new ElapsedTime();
        barTimer = new ElapsedTime();
        hangTimer = new ElapsedTime();
        slideTimer = new ElapsedTime();
    }

    public void powerPID(double power) {
        double posErr = currentTarget - upperSlides.getCurrentPosition(); // measure error in terms of distance between current position and target
        if (Math.abs(posErr) > 5) {
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            double VKd = 0.02;
            lowerSlides.setPower((posErr * VKp + VKd) * power); //instead of fixed power, use the concept of PID and increase power in proportion with the error
            upperSlides.setPower((posErr * VKp + VKd) * power);
        }
    }

    boolean leftBumperDown;


    public void initV4B(LinearOpMode opMode) {
        v4B.init(opMode);
    }

    // Viper Slide Loop
    public void runArm(GamepadEx gamepad1, GamepadEx gamepad2) {
        v4B.updatePosAll(); // Update Virtual Axon Servo Position for PID control
        int positionIncrement = 50;
        switch (slideState) {
            case REST:
                liftMultiplier = 1; // Normal drive speed

                // Account for slippage and prevent motor stalling
                if (limitSwitch.isPressed()) {
                    lowerSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.FIRST;
                    v4B.hang();
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    currentTarget = 1800;
                    slideState = SlideState.HANG;
                }
                if (timer.seconds() > 0.5) {
                    currentTarget = 0;
                }
                break;
            case FIRST:
                currentTarget = firstLvl;
                liftMultiplier = 0.5;
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = secondLvl;
                    slideState = SlideState.SECOND;
                } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    v4B.retract();
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case SECOND:
                liftMultiplier = 0.4;
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = thirdLvl;
                    slideState = SlideState.THIRD;
                } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    v4B.retract();
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case THIRD:
                liftMultiplier = 0.3;
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = SLIDE_LIMIT;
                    slideState = SlideState.FOURTH;
                } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
                    // Hanging mode
                    hanging = true;
                    slideState = SlideState.HANG;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    v4B.retract();
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case FOURTH:
                liftMultiplier = 0.2;
                if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.MANUAL;
                } else if (gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
                    // Hanging mode
                    hanging = true;
                    slideState = SlideState.HANG;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    v4B.retract();
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case MANUAL:
                if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER) && currentTarget > 100) {
                    currentTarget = upperSlides.getCurrentPosition() - positionIncrement;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentTarget = nextLvl();
                } else if (gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
                    // Hanging mode
                    hanging = true;
                    slideState = SlideState.HANG;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    v4B.retract();
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
            case HANG:
                if (gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
                    hangingMultiplier = 0.8;
                    hangRelease = false;
                    lowerSlides.setPower(-1*hangingMultiplier);
                    upperSlides.setPower(-1*hangingMultiplier);
                }
                if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    hangingMultiplier = 0.3;
                    hangTimer.reset();
                    hangRelease = true;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    hanging = false;
                    hangRelease = false;
                    hangingMultiplier = 1;
                    currentTarget = 0;
                    slideState = SlideState.REST;
                }

                if (hangRelease) {
                    lowerSlides.setPower(-hangingMultiplier);
                    upperSlides.setPower(-hangingMultiplier);
                    v4B.hangRest();
                    if (hangTimer.seconds() > 1 && upperSlides.getPower() < 0) {
                        hangingMultiplier -= 0.05;
                        hangTimer.reset();
                    }
                    if (hangingMultiplier <= 0) {
                        hangingMultiplier = 0;
                    }
                }
                break;
        }
        // Code to run the virtual four bar. Don't move in REST state
        if (slideState != SlideState.REST && slideState != SlideState.HANG) {
            if (gamepad1.wasJustPressed(GamepadKeys.Button.A) && !drop) {
                v4B.extend();
                drop = true;
            } else if (gamepad1.wasJustPressed(GamepadKeys.Button.A) && drop) {
                v4B.retract();
                drop = false;
            } else if (gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
                v4B.retract();
            }
        }
        // Set power to the virtual four bar using PID
        if (gamepad1.isDown(GamepadKeys.Button.BACK)) {
            v4B.resetToRest();
        } else {
            v4B.servoPID();
        }
        // Disable PID when hanging
        if (!hanging) {
            powerPID(0.6);
        }
    }

    int nextLvl() {
        double percentage = (double)upperSlides.getCurrentPosition()/SLIDE_LIMIT;
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
        } else {
            return firstLvl;
        }
    }
    // Decrease driving speed for more control when the slides are lifted
    public double getPowerMultiplier() { return liftMultiplier; }

    // Outtake Loop
    public void runOuttake(GamepadEx gamepad1) {
        rtReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ltReader = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (outtakeState) {
            case IDLE:
                if (rtReader.isDown()) {
                    outtakeState = OuttakeState.IN;
                    outtake.setPower(intakePower);
//                } else if (ltReader.isDown() || boxSwitch.isPressed()) {
                } else if (ltReader.isDown()) {
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
                outtake.setPower(-intakePower);
//                if (!ltReader.isDown() && !boxSwitch.isPressed()) {
                if (!ltReader.isDown()) {
                    outtakeState = OuttakeState.IDLE;
                    outtake.setPower(0);
                }
                break;
        }
    }

    // Telemetry
    public boolean leftBumperDown() { return leftBumperDown; }
    public String getLiftState() { return slideState.name(); }
    public int getSlidePosition() { return upperSlides.getCurrentPosition(); }
    public int getCurrentTarget() { return currentTarget; }
    public double getArmTimer() { return timer.seconds(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public double getSlide1Power() { return upperSlides.getPower(); }
    public double getSlide2Power() { return upperSlides.getPower(); }
    public double getV4BErrorRight() { return v4B.getRightError(); }
    public double getV4BErrorLeft() { return v4B.getLeftError(); }

    // Autonomous Functions
    public boolean touching() { return boxSwitch.isPressed(); }
    private boolean dropped = false;

    public Action spinOuttake(double power, double duration) {
        return new Action() {
            private boolean set = false;
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

    public Action readySlides(boolean high) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    slideTimer.reset();
                    set = true;
                }
                if (high) {
                    currentTarget = 450;
                } else {
                    currentTarget = 250;
                }
                powerPID(0.6);
                return !dropped && slideTimer.seconds() < 1.5;
            }
        };
    }

    public Action readySlides(int height) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    lowerSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lowerSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slideTimer.reset();
                    set = true;
                    dropped = false;
                }
                currentTarget = height;
                powerPID(0.6);
                return !dropped && slideTimer.seconds() < 1.5;
            }
        };
    }

    public Action resetSlides() {
        return telemetryPacket -> {
            currentTarget = 0;
            powerPID(0.4);
            if (limitSwitch.isPressed()) {
                VKp = 0.003;
            }
            return !limitSwitch.isPressed();
        };
    }

    public Action ready4bar() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    barTimer.reset();
                    set = true;
                }
                v4B.updatePosAll();
                v4B.extend(); // Extend
                v4B.servoPID();
                return barTimer.seconds() < 2;
            }
        };
    }

    public Action reset4Bar() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    barTimer.reset();
                    set = true;
                }
                v4B.updatePosAll();
                v4B.retract(); // Extend
                v4B.servoPID();
                if (opMode != null) {
                    opMode.telemetry.addData("Bar Timer:", barTimer.seconds());
                    opMode.telemetry.addData("Slide Reset Completed", limitSwitch.isPressed());
                    opMode.telemetry.update();
                }
                if (barTimer.seconds() > 1.9) {
                    dropped = true;
                }
                return barTimer.seconds() < 2;
            }
        };
    }

    public Action resetToRest() {
        return new ParallelAction(
                reset4Bar(),
                resetSlides()
        );
    }

    public Action dropYellowPixel(boolean high) {
        return new SequentialAction (
                new ParallelAction(
                        readySlides(high),
                        ready4bar()
                ),
                spinOuttake(-0.5, 1.5)
        );
    }

    public Action dropWhitePixel() {
        return new SequentialAction (
                new SleepAction(5),
                new ParallelAction(
                        readySlides(600),
                        ready4bar()
                ),
                spinOuttake(-0.5, 2),
                reset4Bar(),
                resetSlides()
        );
    }
}
