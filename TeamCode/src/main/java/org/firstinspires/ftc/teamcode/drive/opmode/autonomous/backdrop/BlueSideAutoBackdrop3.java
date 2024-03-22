package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.backdrop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.AutoCoordinates;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem4;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Autonomous(name = "BLUE Backdrop ADVANCED", group = "Backdrop Side")
public class BlueSideAutoBackdrop3 extends LinearOpMode {

    BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Get the coordinates
        AutoCoordinates coords = new AutoCoordinates(false, false);

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmSubsystem4 arm = new ArmSubsystem4(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();

        // Initialize some functions
        AutoFunctions functions = new AutoFunctions();
        functions.init(new IntakeSubsystem(hardwareMap), arm, drive, true);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propLeftPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromLeftProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backdropLeftPos, coords.ROTATED)
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.propCenterPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromCenterProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backdropCenterPos, coords.ROTATED)
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propRightPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromRightProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backdropRightPos, coords.ROTATED)
                .build();

        // To Pixel Stack after Drop
        Action leftToPixel = drive.actionBuilder(new Pose2d(coords.afterDropLeft, coords.ROTATED))
                .strafeToConstantHeading(coords.rotatedStartPos)
                .strafeToConstantHeading(coords.crossTrussPos)
                .strafeToConstantHeading(coords.pixelStackPosClose)
                .build();
        Action middleToPixel = drive.actionBuilder(new Pose2d(coords.afterDropCenter, coords.ROTATED))
                .strafeToConstantHeading(coords.rotatedStartPos)
                .strafeToConstantHeading(coords.crossTrussPos)
                .strafeToConstantHeading(coords.pixelStackPosClose)
                .build();
        Action rightToPixel = drive.actionBuilder(new Pose2d(coords.afterDropRight, coords.ROTATED))
                .strafeToConstantHeading(coords.rotatedStartPos)
                .strafeToConstantHeading(coords.crossTrussPos)
                .strafeToConstantHeading(coords.pixelStackPosClose)
                .build();

        Action intakePixelDrive = drive.actionBuilder(new Pose2d(coords.pixelStackPosClose, coords.ROTATED))
                .strafeToLinearHeading(coords.backIntoPixelPosClose, coords.ROTATED)
                .strafeToLinearHeading(coords.backToIntakePixelClose, coords.ROTATED)
                .strafeToLinearHeading(coords.pixelStackPosClose, coords.ROTATED)
                .strafeToLinearHeading(coords.backIntoPixelPosClose, coords.ROTATED)
                .strafeToLinearHeading(coords.pixelStackPosClose, coords.ROTATED)
                .strafeToLinearHeading(coords.backToIntakePixelClose, coords.ROTATED)
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

        Action runParkPath = middleToPixel;
        Vector2d whiteDropoffPos = coords.cringerBackdropCenterPos;

        // Stop the pipeline since we no longer need to detect the prop
        CVMediator.visionPortal.setProcessorEnabled(octopusPipeline, false);

        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                whiteDropoffPos = coords.cringerBackdropPosNotCenter;
                Actions.runBlocking(new ParallelAction(
                        runToCenterProp,
                        new SequentialAction(
                                new SleepAction(3),
                                new ParallelAction(
                                        arm.readySlides(false),
                                        arm.ready4bar()
                                ),
                                arm.spinOuttake(-0.5, 1.5)
                        )
                ));
                break;
            case LEFT:
                runParkPath = leftToPixel;
                Actions.runBlocking(new ParallelAction(
                        runToLeftProp,
                        new SequentialAction(
                                new SleepAction(4),
                                new ParallelAction(
                                        arm.readySlides(false),
                                        arm.ready4bar()
                                ),
                                arm.spinOuttake(-0.5, 1.5)
                        )
                ));
                break;
            case RIGHT:
                runParkPath = rightToPixel;
                Actions.runBlocking(new ParallelAction(
                        runToRightProp,
                        new SequentialAction(
                                new SleepAction(4),
                                new ParallelAction(
                                        arm.readySlides(false),
                                        arm.ready4bar()
                                ),
                                arm.spinOuttake(-0.5, 1.5)
                        )
                ));
                break;
        }
        Action whiteToBackdrop = drive.actionBuilder(new Pose2d(coords.backIntoPixelPosClose, coords.ROTATED))
                .strafeToConstantHeading(coords.crossTrussPos)
                .strafeToConstantHeading(coords.rotatedStartPos)
                .strafeToConstantHeading(whiteDropoffPos)
                .build();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        arm.reset4Bar(),
                        arm.resetSlides(),
                        runParkPath
                ),
                new ParallelAction(
                        functions.intakePixelClose(),
                        intakePixelDrive
                ),
                new ParallelAction(
                        intake.spinIntake(-0.8, 2),
                        arm.spinOuttake(0.8, 2),
                        whiteToBackdrop,
                        arm.dropWhitePixel()
                )
        ));
    }
}
