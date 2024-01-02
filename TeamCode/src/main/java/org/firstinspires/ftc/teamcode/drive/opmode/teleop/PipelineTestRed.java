package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;

@Disabled
@TeleOp(name = "Pipeline Test Red", group = "CV")
public class PipelineTestRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RedOctopusPipeline octopusPipeline = new RedOctopusPipeline();
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();
        CVMediator.init(hardwareMap, null, octopusPipeline, false, this);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Zone", octopusPipeline.getLocation());
            telemetry.addData("Pixels in left", octopusPipeline.getLeftSum());
            telemetry.addData("Pixels in middle", octopusPipeline.getCenterSum());
            telemetry.addData("Pixels in right", octopusPipeline.getRightSum());
            telemetry.update();
        }
    }
}