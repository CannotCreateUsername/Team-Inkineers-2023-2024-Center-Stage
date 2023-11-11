package org.firstinspires.ftc.teamcode.drive;


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

    public final int DROP = -1;
    public final int LOAD = 1;

    SlideState slideState;
    OuttakeState outtakeState;

    private double currentTarget;
    private boolean reversed = false;

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

        virtualBar.setPosition(LOAD);

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
        int SLIDE_LIMIT = 1800;
        switch (slideState) {
            case REST:
                if (gamepad1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.READY;
                    timer.reset();
                }
                if (timer.seconds() > 1.5) {
                    runToPosition(50);
                }
                break;
            case READY:
                runToPosition(200, 0.2);
                if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
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
                if (gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER) && slides.getCurrentPosition() < SLIDE_LIMIT) {
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
                if (gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.RUNNING;
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    virtualBar.setPosition(LOAD);
                    slideState = SlideState.REST;
                    timer.reset();
                }
                break;
        }
        if (gamepad1.wasJustPressed(GamepadKeys.Button.X)) {
            reversed = !reversed;
        }
        gamepad1.readButtons();
    }

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
    public String getLiftState() { return slideState.name(); }
    public int getSlidePosition() { return slides.getCurrentPosition(); }
    public double getV4bPosition() { return virtualBar.getPosition(); }
    public double getArmTimer() { return timer.seconds(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public boolean isReversed() { return reversed; }

    // Autonomous Functions

    public Action dropYellowPixel() {
        ElapsedTime dropTimer = new ElapsedTime();
        return telemetryPacket -> {
            dropTimer.reset();
            runToPosition(200);
            virtualBar.setPosition(DROP);
            outtake.setPower(-1);
            return dropTimer.seconds() < 2;
        };
    }
}
