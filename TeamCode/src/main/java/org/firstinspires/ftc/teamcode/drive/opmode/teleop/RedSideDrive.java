package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.GamepadHelper;
import org.firstinspires.ftc.teamcode.drive.IntakeSubsystem;

@TeleOp(name = "Red Drive", group = "Linear Opmode")
public class RedSideDrive extends LinearOpMode {
    enum TurnState {
        STRAIGHT,
        ROTATED
    }
    TurnState turnState;

    double leftXInput;
    double leftYInput;
    double rightXInput;

    @Override
    public void runOpMode() throws InterruptedException {
        turnState = TurnState.ROTATED;

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        // Initialize the drive code
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        // Initialize intake code
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // Initialize arm code
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);

        // Initialize speed ramping
        GamepadHelper leftStickX = new GamepadHelper();
        leftStickX.init();
        GamepadHelper leftStickY = new GamepadHelper();
        leftStickY.init();
        GamepadHelper rightStickX = new GamepadHelper();
        rightStickX.init();

        waitForStart();
        while (opModeIsActive()) {
            switch (turnState) {
                case ROTATED:
                    leftXInput = leftStickY.getRampingValue(gamepad1.left_stick_y);
                    leftYInput = -leftStickX.getRampingValue(gamepad1.left_stick_x);
                    rightXInput = rightStickX.getRampingValue(gamepad1.right_stick_x);
                    if (gamepad1.y) {
                        turnState = TurnState.STRAIGHT;
                    }
                    break;
                case STRAIGHT:
                    leftXInput = leftStickX.getRampingValue(gamepad1.left_stick_x);
                    leftYInput = leftStickY.getRampingValue(gamepad1.left_stick_y);
                    rightXInput = rightStickX.getRampingValue(gamepad1.right_stick_x);
                    if (gamepad1.x) {
                        turnState = TurnState.ROTATED;
                    }
                    break;
            }

            // Take gamepad input and pass it into the mecanum drive function
            drive.setDrivePowers(new PoseVelocity2d
                    (new Vector2d(
                            -leftYInput,
                            -leftXInput),
                            -rightXInput
                    )
            );

            // Intake control loop
            intakeSubsystem.runIntake(gamepadEx1);
            // Arm control loop
            armSubsystem.runArm(gamepadEx1);
            armSubsystem.runOuttake(gamepadEx2);

            // Telemetry
            telemetry.addData("Intake State", intakeSubsystem.getIntakeState());
            telemetry.addData("Turn State", turnState.name());
            telemetry.addData("Lift State", armSubsystem.getLiftState());
            telemetry.addData("Load State", armSubsystem.getLoadState());
            telemetry.update();
        }
    }
}
