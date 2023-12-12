package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveFunctions;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.EndgameSubsystems;
import org.firstinspires.ftc.teamcode.drive.GamepadHelper;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@TeleOp(name = "Blue Drive", group = "Linear Opmode")
public class BlueSideDrive extends LinearOpMode {
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
        DriveFunctions functions = new DriveFunctions(hardwareMap, drive, this);

        // Initialize intakeSubsystem code
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        // Initialize arm code
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        //Initialize drone launcher and hanging code
        EndgameSubsystems endgame = new EndgameSubsystems(hardwareMap);

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
                    leftXInput = -leftStickY.getRampingValue(gamepad1.left_stick_y);
                    leftYInput = leftStickX.getRampingValue(gamepad1.left_stick_x);
                    rightXInput = rightStickX.getRampingValue(gamepad1.right_stick_x);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.Y)) {
                        functions.turnPID(-90);
                        turnState = TurnState.STRAIGHT;
                    }
                    break;
                case STRAIGHT:
                    leftXInput = leftStickX.getRampingValue(gamepad1.left_stick_x);
                    leftYInput = leftStickY.getRampingValue(gamepad1.left_stick_y);
                    rightXInput = rightStickX.getRampingValue(gamepad1.right_stick_x);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.Y)) {
                        functions.turnPID(90);
                        turnState = TurnState.ROTATED;
                    }
                    break;
            }
            leftXInput *= arm.getPowerMultiplier();
            leftYInput *= arm.getPowerMultiplier();
            rightXInput*= arm.getPowerMultiplier();

            // Take gamepad input and pass it into the mecanum drive function
            drive.setDrivePowers(new PoseVelocity2d
                    (new Vector2d(
                            -leftYInput,
                            -leftXInput),
                            -rightXInput
                    )
            );

            // Run Robot Subsystems
            // Arm control loop
            arm.runArm(gamepadEx1, gamepadEx2);
            arm.runOuttake(gamepadEx1);
            // Intake control loop
            intake.runIntake(gamepadEx1);
            // Endgame control loop
            endgame.run(gamepadEx2);

            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Telemetry
            telemetry.addData("Turn State", turnState.name());
            telemetry.addData("Lift State", arm.getLiftState());
            telemetry.addData("Drone Launch State", endgame.getLauncherState());
            telemetry.addData("Drive Multiplier", arm.getPowerMultiplier());
            telemetry.update();
        }
    }
}
