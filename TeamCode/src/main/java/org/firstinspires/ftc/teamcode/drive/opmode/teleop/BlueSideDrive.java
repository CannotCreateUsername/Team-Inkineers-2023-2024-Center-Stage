package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.IntakeSubsystem;

@TeleOp(name = "Whats under there", group = "Linear Opmode")
public class BlueSideDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        // Initialize the drive code
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        // Initialize intake code
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // Initialize arm code
        //ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            // Take gamepad input and pass it into the mecanum drive function
            drive.setDrivePowers(new PoseVelocity2d
                    (new Vector2d(
                            -gamepad1.left_stick_y*.8,
                            -gamepad1.left_stick_x*.8),
                            -gamepad1.right_stick_x*.8
                    )
            );

            // Intake control loop
            intakeSubsystem.runIntake(gamepadEx1);
            // Arm control loop
            //armSubsystem.run(gamepadEx1);

            // Telemetry
            telemetry.addData("DEEZ NUTS", "under-where!? xdddDDddddd lol.");
            telemetry.addData("Intake State", intakeSubsystem.getIntakeState());
            //telemetry.addData("Lift State", armSubsystem.getLiftState());
            //telemetry.addData("Load State", armSubsystem.getLoadState());
            telemetry.update();
        }
    }
}
