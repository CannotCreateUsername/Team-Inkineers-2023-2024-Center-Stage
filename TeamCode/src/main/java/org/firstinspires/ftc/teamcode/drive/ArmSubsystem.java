package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem {
    public enum SlideState {
        TOP,
        MIDDLE,
        BOTTOM,
        REST
    }
    public enum DropState {
        DROP_OFF,
        PICK_UP
    }

    private final static double Kp = .05;

    private DcMotor slides = null;
    private Servo virtualBar = null;

    SlideState slideState;
    DropState dropState;

    double currentTarget;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        virtualBar = hardwareMap.get(Servo.class, "bar");

        slideState = SlideState.REST;
        dropState = DropState.PICK_UP;
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

    public void run(GamepadEx gamepad1) {
        switch (slideState) {
            case REST:
                runToPosition(0);
                if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.BOTTOM;
                }
            case BOTTOM:
                runToPosition(100);
                if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.MIDDLE;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.REST;
                }
            case MIDDLE:
                runToPosition(200);
                if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.TOP;
                } else if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.BOTTOM;
                }
            case TOP:
                runToPosition(300);
                if (gamepad1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    slideState = SlideState.BOTTOM;
                }
        }

        switch (dropState) {
            case PICK_UP:
                virtualBar.setPosition(1);
                if (gamepad1.isDown(GamepadKeys.Button.X)) {
                    dropState = DropState.DROP_OFF;
                }
            case DROP_OFF:
                virtualBar.setPosition(0);
                if (gamepad1.wasJustReleased(GamepadKeys.Button.X)) {
                    dropState = DropState.PICK_UP;
                }
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

    public String getLiftState() { return slideState.name(); }
    public String getLoadState() { return dropState.name(); }
}
