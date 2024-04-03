package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.substation;

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
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.AutoCoordinates;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem3;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Autonomous(name = "RED Substation WAIT", group = "Substation Side")
public class RedSideAutoSubstation3 extends LinearOpMode {

    RedOctopusPipeline octopusPipeline = new RedOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        AutoCoordinates coords = new AutoCoordinates(true, true);

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmSubsystem3 arm = new ArmSubsystem3(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();

        // Initialize some functions
        AutoFunctions functions = new AutoFunctions();
        functions.init(intake, arm, drive);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propLeftPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.beforePixelCrash, coords.ROTATED)
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.propCenterPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromCenterProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.beforePixelCrash, coords.ROTATED)
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propRightPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.beforePixelCrash, coords.ROTATED)
                .build();
        // Run to the pixel (spin intake at same time)
        Action runToPixelStack = drive.actionBuilder(new Pose2d(coords.beforePixelCrash, coords.ROTATED))
                .strafeToLinearHeading(coords.pixelStackPosFar, coords.ROTATED)
                .strafeToLinearHeading(coords.backIntoPixelPosFar, coords.ROTATED)
                .strafeToLinearHeading(coords.backToIntakePixelFar, coords.ROTATED)
                .strafeToLinearHeading(coords.pixelStackPosFar, coords.ROTATED)
                .strafeToLinearHeading(coords.backToIntakePixelFar, coords.ROTATED)
                .strafeToLinearHeading(coords.pixelStackPosFar, coords.ROTATED)
                .build();

        Action runAcrossField = drive.actionBuilder(new Pose2d(coords.pixelStackPosFar, coords.ROTATED))
                .splineTo(coords.toBackdropFromPixelStack, coords.ROTATED_AF)
                .build();

        CVMediator.init(hardwareMap, drive, octopusPipeline, true, this);

        // Display Telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Detection", octopusPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();
        timer1.reset();
        if (isStopRequested()) return;
        // OPMODE STARTS HERE
        CVMediator.visionPortal.setProcessorEnabled(octopusPipeline, false);
        CVMediator.disableAprilTag();

        // Store which path to take (Default Middle)
        Vector2d dropWhitePos = coords.subLeftBackdrop;
        Vector2d dropYellowPos = coords.subCenterBackdrop;

        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                Actions.runBlocking(new SequentialAction(
                        runToCenterProp,
                        new ParallelAction(
                                runToPixelStack,
                                functions.intakePixelFar()
                        )
                ));
                break;
            case LEFT:
                dropWhitePos = coords.subCenterBackdrop;
                dropYellowPos = coords.subLeftBackdrop;
                Actions.runBlocking(new SequentialAction(
                        runToLeftProp,
                        new ParallelAction(
                                runToPixelStack,
                                functions.intakePixelFar()
                        )
                ));
                break;
            case RIGHT:
                dropWhitePos = coords.subCenterBackdrop;
                dropYellowPos = coords.subRightBackdrop;
                Actions.runBlocking(new SequentialAction(
                        runToRightProp,
                        new ParallelAction(
                                runToPixelStack,
                                functions.intakePixelFar()
                        )
                ));
                break;
        }

        Action runToScoreWhite = drive.actionBuilder(new Pose2d(coords.toBackdropFromPixelStack, coords.ROTATED))
                .strafeToLinearHeading(dropWhitePos, coords.ROTATED)
                .build();
        Action runToScoreYellow = drive.actionBuilder(new Pose2d(dropWhitePos, coords.ROTATED))
                .strafeToLinearHeading(coords.betweenSubBackdrop, coords.ROTATED)
                .strafeToLinearHeading(dropYellowPos, coords.ROTATED)
                .build();
        Action park = drive.actionBuilder(new Pose2d(dropYellowPos, coords.ROTATED))
                .strafeToLinearHeading(coords.subParkPos, coords.ROTATED)
                .build();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        runAcrossField,
                        intake.spinIntake(-0.8, 3),
                        new SequentialAction(
                                new SleepAction(2),
                                arm.readySlides(true)
                        )
                ),
                CVMediator.waitForClear(), // Wait for alliance to get out of the way
                new ParallelAction(
                        runToScoreWhite,
                        arm.ready4bar()
                ),
                arm.spinOuttake(-0.5, 0.5),
                new ParallelAction(
                        runToScoreYellow,
                        new SequentialAction(
                                new SleepAction(1),
                                arm.readySlides(true)
                        )
                ),
                arm.spinOuttake(-1, 0.6),
                new ParallelAction(
                        arm.reset4Bar(),
                        arm.resetSlides(),
                        park
                )
        ));
    }
}
