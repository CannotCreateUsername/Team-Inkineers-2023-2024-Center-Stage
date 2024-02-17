package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;

@Disabled
@TeleOp(name = "Dual CV Testing", group = "CV")
public class DualCVTester extends LinearOpMode {

    MecanumDrive drive;
    ComputerVisionMediator computerVisionMediator = new ComputerVisionMediator();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        computerVisionMediator.init(hardwareMap, drive, new RedOctopusPipeline(), true, this);

        ElapsedTime cringeTimer = new ElapsedTime();

        while (!opModeIsActive() && !isStopRequested()) {
            computerVisionMediator.telemetryAprilTag(this);
            telemetry.update();
        }

        waitForStart();
        cringeTimer.reset();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                Actions.runBlocking(computerVisionMediator.turn90(false));
            } else if (gamepad1.b) {
                Actions.runBlocking(computerVisionMediator.turn90(true));
            }

            // Take gamepad input and pass it into the mecanum drive function
            drive.setDrivePowers(new PoseVelocity2d
                    (new Vector2d(
                            -gamepad1.left_stick_x,
                            -gamepad1.left_stick_y),
                            -gamepad1.right_stick_x
                    )
            );

            telemetry.addData("Yaw Angle", computerVisionMediator.getYawAngle());

            computerVisionMediator.telemetryAprilTag(this);

//            telemetry.addLine("Never gonna");
//
//            if (cringeTimer.seconds() > 1) {
//                telemetry.addLine("give you up");
//            }
//            if (cringeTimer.seconds() > 1.5) {
//                telemetry.addLine("Never gonna");
//            }
//            if (cringeTimer.seconds() > 2) {
//                telemetry.addLine("let you down");
//            }
//            if (cringeTimer.seconds() > 2.5) {
//                telemetry.addLine("Never gonna");
//            }
//            if (cringeTimer.seconds() > 3) {
//                telemetry.addLine("turn around");
//            }
//            if (cringeTimer.seconds() > 3.5) {
//                telemetry.addLine("And hurt you");
//            }
//            if (cringeTimer.seconds() > 5) {
//                telemetry.addLine("AJSHFDNjfasjkbnsdagjkrsrngvrnKLNFKLSVKLKLNVklfdngjkrgsEKFiy489t5u89iojklJMEKDFMK:m3mtFLEMSDJLENFksejmkVMSD:LPLF{#PREPF{KFOI&T*$&%Yuiher%KEshdrfbsodiSHT$BIJKsernhfjksh5tjkz5N$JKRKNGNrKTN4wt");
//            }
            telemetry.update();
        }
    }
}
