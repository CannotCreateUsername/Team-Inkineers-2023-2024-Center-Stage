package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem4;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem5;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

import javax.annotation.Nullable;

public class AutoFunctions {
    private ArmSubsystem4 arm;
    private ArmSubsystem5 arm2;
    private IntakeSubsystem intake;
    private MecanumDrive drive;
    ElapsedTime moveTimer;

    public final double DURATION_WHITE = 0.38;
    public final double DURATION_YELLOW = 0.6;

    public void init(@Nullable IntakeSubsystem intakeSubsystem, ArmSubsystem4 armSubsystem, MecanumDrive mecanumDrive) {
        intake = intakeSubsystem;
        arm = armSubsystem;
        arm2 = null;
        drive = mecanumDrive;
        moveTimer = new ElapsedTime();
    }

    public void init(@Nullable IntakeSubsystem intakeSubsystem, ArmSubsystem5 armSubsystem, MecanumDrive mecanumDrive) {
        intake = intakeSubsystem;
        arm2 = armSubsystem;
        arm = null;
        drive = mecanumDrive;
        moveTimer = new ElapsedTime();
    }

    public Action intakePixelFar() {
        return new SequentialAction(
                intake.spinIntake(-0.2, 2.5),
                new ParallelAction(
                        intake.spinIntake(0.6, 3),
                        arm == null ? arm2.spinOuttake(0.6, 3) : arm.spinOuttake(0.6, 3)
                ),
                intake.spinIntake(-0.8, 0.5),
                arm == null ? arm2.spinOuttake(-0.05, 0.2) : arm.spinOuttake(-0.05, 0.4)
        );
    }

    public Action intakePixelClose() {
        return new SequentialAction(
                intake.spinIntake(1, 0.5),
                new ParallelAction(
                        intake.spinIntake(0.8, 4),
                        arm == null ? arm2.spinOuttake(0.8, 4) : arm.spinOuttake(0.8, 4)
                ),
                arm == null ? arm2.spinOuttake(-0.05, 0.2) : arm.spinOuttake(-0.05, 0.4)
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
                if (!arm2.touching() && moveTimer.seconds() < 2.5) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0.1, 0), Math.toRadians(0)
                    ));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0, 0), Math.toRadians(0)
                    ));
                }
                if (moveTimer.seconds() > 2.5) {
                    return false;
                } else {
                    return (!arm2.touching());
                }
            }
        };
    }
}
