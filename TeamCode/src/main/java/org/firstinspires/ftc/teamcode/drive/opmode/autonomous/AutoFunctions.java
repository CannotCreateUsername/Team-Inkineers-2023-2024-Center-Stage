package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem2;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

public class AutoFunctions {
    private ArmSubsystem2 arm;
    private IntakeSubsystem intake;
    private MecanumDrive drive;
    private boolean isOnBlueSide;

    public void init(IntakeSubsystem intakeSubsystem, ArmSubsystem2 armSubsystem, MecanumDrive mecanumDrive, boolean blueSide) {
        intake = intakeSubsystem;
        arm = armSubsystem;
        drive = mecanumDrive;
        isOnBlueSide = blueSide;
    }

    public void intakePixel(Pose2d startPose) {
        Action moveToStack = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(40, isOnBlueSide?26:-26))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                moveToStack,
                                intake.spinIntake(0.5, 4),
                                arm.spinOuttake(-0.5, 4)
                        ),
                        intake.spinIntake(-0.5, 2)
                )
        );
    }
}
