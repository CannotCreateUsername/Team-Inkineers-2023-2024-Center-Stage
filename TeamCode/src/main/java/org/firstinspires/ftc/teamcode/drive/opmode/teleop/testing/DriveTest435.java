package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.GamepadHelper;

@TeleOp(name = "Drive Test 435", group = "Linear Opmode")
public class DriveTest435 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        // Initialize speed ramping
        GamepadHelper leftStickX = new GamepadHelper();
        leftStickX.init();
        GamepadHelper leftStickY = new GamepadHelper();
        leftStickY.init();
        GamepadHelper rightStickX = new GamepadHelper();
        rightStickX.init();

        waitForStart();
        while (opModeIsActive()) {
            double leftYInput = -leftStickY.getRampingValue(gamepad1.left_stick_y);
            double leftXInput = leftStickX.getRampingValue(gamepad1.left_stick_x);
            double rightXInput = rightStickX.getRampingValue(gamepad1.right_stick_x);
            // Take gamepad input and pass it into the mecanum drive function
            drive.setDrivePowers(new PoseVelocity2d
                    (new Vector2d(
                            -leftYInput,
                            -leftXInput),
                            -rightXInput
                    )
            );
        }

    }
}
