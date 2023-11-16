package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.EndgameSubsystems;
import org.firstinspires.ftc.teamcode.drive.IntakeSubsystem;

@TeleOp(name = "Arm Testing", group = "Linear opmode")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        EndgameSubsystems endgameSubsystems = new EndgameSubsystems(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                Actions.runBlocking(new SequentialAction(
                        armSubsystem.dropYellowPixel()
                ));
            }

            armSubsystem.runArm(gamepadEx1);
            armSubsystem.runOuttake(gamepadEx1);
            intakeSubsystem.runIntake(gamepadEx1);
            endgameSubsystems.run(gamepadEx1);

            telemetry.addData("Slide State", armSubsystem.getLiftState());
            telemetry.addData("Slide Position", armSubsystem.getSlidePosition());
            telemetry.addData("V4B Position", armSubsystem.getV4bPosition());
            telemetry.addData("Arm Timer", armSubsystem.getArmTimer());
            telemetry.addData("Outtake State", armSubsystem.getOuttakeState());
            telemetry.addData("Intake State", intakeSubsystem.getIntakeState());
            telemetry.addData("Reversed?", armSubsystem.isReversed());
            telemetry.addData("Drone Launch State", endgameSubsystems.getLauncherState());
            telemetry.addData("Right Bumper Down?", armSubsystem.rightBumperDown());
            telemetry.update();
        }
    }
}
