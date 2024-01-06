package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Autonomous(name = "Red Alliance Backdrop Auto", group = "Backdrop Side")
public class RedSideAutoBackdrop extends LinearOpMode {

    RedOctopusPipeline octopusPipeline = new RedOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();

        // Initialize some functions
        AutoFunctions functions = new AutoFunctions();
        functions.init(new IntakeSubsystem(hardwareMap), arm, drive, false);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(28, 0))
                .strafeToConstantHeading(new Vector2d(28, -11))
                .strafeToConstantHeading(new Vector2d(24, -11))
                .strafeToConstantHeading(new Vector2d(31, 31))
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(31, 0))
                .strafeToConstantHeading(new Vector2d(24, 0))
                .strafeToConstantHeading(new Vector2d(26, 32))
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(28, 0))
                .strafeToConstantHeading(new Vector2d(28, 12))
                .strafeToConstantHeading(new Vector2d(24, 12))
                .strafeToConstantHeading(new Vector2d(18, 34))
                .build();

        Action scoot = drive.actionBuilder(new Pose2d(new Vector2d(24, 34), Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(24, 28))
                .build();

        // Park in backstage
        Action rightPark = drive.actionBuilder(new Pose2d(new Vector2d(24, 34), Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-8, 34))
                .build();
        Action middlePark = drive.actionBuilder(new Pose2d(new Vector2d(24, 30), Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(4, 36))
                .build();

        // Initialize all computer vision stuff
        CVMediator.init(hardwareMap, drive, octopusPipeline, false, this);

        // Display Telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Detection", octopusPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();
        timer1.reset();
        if (isStopRequested()) return;

        // Stop the pipeline since we no longer need to detect the prop
        CVMediator.visionPortal.setProcessorEnabled(octopusPipeline, false);


        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                Actions.runBlocking(runToCenterProp);
                CVMediator.turnPID(-90);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoot,
                                arm.dropYellowPixel()
                        ),
                        middlePark
                ));
                break;
            case LEFT:
                Actions.runBlocking(runToLeftProp);
                CVMediator.turnPID(-90);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoot,
                                arm.dropYellowPixel()
                        ),
                        rightPark
                ));
                break;
            case RIGHT:
                Actions.runBlocking(runToRightProp);
                CVMediator.turnPID(-90);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoot,
                                arm.dropYellowPixel()
                        ),
                        middlePark
                ));
                break;
        }
    }
}
