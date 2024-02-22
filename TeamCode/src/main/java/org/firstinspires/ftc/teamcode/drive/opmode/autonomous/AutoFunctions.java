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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem3;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

import javax.annotation.Nullable;

public class AutoFunctions {
    private ArmSubsystem3 arm;
    private IntakeSubsystem intake;
    private MecanumDrive drive;
    private boolean isOnBlueSide;
    ElapsedTime moveTimer;


    public void init(@Nullable IntakeSubsystem intakeSubsystem, ArmSubsystem3 armSubsystem, MecanumDrive mecanumDrive, boolean blueSide) {
        intake = intakeSubsystem;
        arm = armSubsystem;
        drive = mecanumDrive;
        isOnBlueSide = blueSide;
        moveTimer = new ElapsedTime();
    }

    public Action intakePixel() {
        return new SequentialAction(
                intake.spinIntake(-0.5, 3),
                new ParallelAction(
                        intake.spinIntake(0.4, 5),
                        arm.spinOuttake(0.4, 5)
                ),
                intake.spinIntake(-0.5, 2)
        );
    }

    public Action touchBackdrop() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    moveTimer.reset();
                    set = true;
                }
                if (!arm.touching() && moveTimer.seconds() < 2.5) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0.1, 0), Math.toRadians(0)
                    ));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0, 0), Math.toRadians(0)
                    ));
                }
                if (moveTimer.seconds() > 2.6) {
                    return false;
                }
                return (!arm.touching());
            }
        };
    }
}
