package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;

@TeleOp(name = "Pipeline Test Blue", group = "CV")
public class PipelineTestBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();
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