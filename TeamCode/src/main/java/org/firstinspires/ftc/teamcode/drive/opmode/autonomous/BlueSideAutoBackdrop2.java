package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.drive.AutoCoordinates;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem3;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Disabled
@Autonomous(name = "Blue Alliance Backdrop Inside", group = "Backdrop Side")
public class BlueSideAutoBackdrop2 extends LinearOpMode {

    BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Get the coordinates
        AutoCoordinates coords = new AutoCoordinates(false);

        // Initialize the drive
        Pose2d startPose = coords.startPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmSubsystem3 arm = new ArmSubsystem3(hardwareMap);
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

        // Park in backstage
        Action leftPark = drive.actionBuilder(new Pose2d(coords.afterDropLeft, coords.ROTATED))
                .strafeToLinearHeading(coords.parkInsidePos, coords.ROTATED)
                .build();
        Action middlePark = drive.actionBuilder(new Pose2d(coords.afterDropCenter, coords.ROTATED))
                .strafeToLinearHeading(coords.parkInsidePos, coords.ROTATED)
                .build();
        Action rightPark = drive.actionBuilder(new Pose2d(coords.afterDropRight, coords.ROTATED))
                .strafeToLinearHeading(coords.parkInsidePos, coords.ROTATED)
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
//                CVMediator.turnPID(90);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                functions.touchBackdrop(),
                                arm.dropYellowPixel(false)
                        ),
                        middlePark
                ));
                break;
            case LEFT:
                Actions.runBlocking(runToLeftProp);
//                CVMediator.turnPID(90);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                functions.touchBackdrop(),
                                arm.dropYellowPixel(false)
                        ),
                        leftPark
                ));
                break;
            case RIGHT:
                Actions.runBlocking(runToRightProp);
//                CVMediator.turnPID(90);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                functions.touchBackdrop(),
                                arm.dropYellowPixel(false)
                        ),
                        rightPark
                ));
                break;
        }
    }
}
