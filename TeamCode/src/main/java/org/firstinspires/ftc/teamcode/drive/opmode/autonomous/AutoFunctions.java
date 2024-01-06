package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

import javax.annotation.Nullable;

public class AutoFunctions {
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private MecanumDrive drive;
    private boolean isOnBlueSide;

    public void init(@Nullable IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, MecanumDrive mecanumDrive, boolean blueSide) {
        intake = intakeSubsystem;
        arm = armSubsystem;
        drive = mecanumDrive;
        isOnBlueSide = blueSide;
    }

    public void intakePixel(Pose2d startPose) {
        Action moveToStack = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(24, isOnBlueSide?26:-26))
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

    public Action touchBackdrop() {
        return telemetryPacket -> {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0.1, 0), Math.toRadians(0)
            ));
            return !arm.touching();
        };
    }
}
