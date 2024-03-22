package org.firstinspires.ftc.teamcode.drive.opmode.teleop.testing;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem4;
import org.firstinspires.ftc.teamcode.drive.subsystems.EndgameSubsystems;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@TeleOp(name = "Arm Testing", group = "Linear opmode")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        ArmSubsystem4 armSubsystem = new ArmSubsystem4(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        EndgameSubsystems endgameSubsystems = new EndgameSubsystems(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                Actions.runBlocking(new SequentialAction(
                        armSubsystem.dropYellowPixel(true)
                ));
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A) && armSubsystem.intakePower <= 1) {
                armSubsystem.hangingMultiplier += 0.1;
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B) && armSubsystem.intakePower >= -1) {
                armSubsystem.hangingMultiplier -= 0.1;
            }

            armSubsystem.runArm(gamepadEx1, gamepadEx2);
            armSubsystem.runOuttake(gamepadEx1);
            intakeSubsystem.runIntake(gamepadEx1);
            endgameSubsystems.run(gamepadEx1, gamepadEx2);

            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            telemetry.addData("Slide State", armSubsystem.getLiftState());
            telemetry.addData("Slide Position", armSubsystem.getSlidePosition());
            telemetry.addData("Current Target", armSubsystem.getCurrentTarget());
//            telemetry.addData("V4B Position", armSubsystem.getV4bPosition());
            telemetry.addData("Arm Timer", armSubsystem.getArmTimer());
            telemetry.addData("Outtake State", armSubsystem.getOuttakeState());
            telemetry.addData("Intake State", intakeSubsystem.getIntakeState());
            telemetry.addData("Drone Launch State", endgameSubsystems.getLauncherState());
//            telemetry.addData("Right Bumper Down?", armSubsystem.rightBumperDown());
            telemetry.addData("FIRST Slide motor power", armSubsystem.getSlide1Power());
            telemetry.addData("SECOND Slide motor power", armSubsystem.getSlide2Power());
            telemetry.addData("Intake Power", armSubsystem.intakePower);
            telemetry.addData("Hanging Power", armSubsystem.hangingMultiplier);
            telemetry.addData("Box Switch Pressed?", armSubsystem.touching());
            telemetry.update();
        }
    }
}
