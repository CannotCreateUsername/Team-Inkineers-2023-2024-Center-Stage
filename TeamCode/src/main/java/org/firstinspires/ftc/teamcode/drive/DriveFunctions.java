package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.IMUKd;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.IMUKp;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveFunctions {
    private final IMU imu;
    private final MecanumDrive drive;
    private final LinearOpMode opMode;

    public DriveFunctions(HardwareMap hardwareMap, MecanumDrive mecanumDrive, LinearOpMode linearOpMode) {
        drive = mecanumDrive;
        opMode = linearOpMode;
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);
    }

    public void turnPID(double degrees) {
        ElapsedTime timer = new ElapsedTime();
        double power;
        double error = 1;

        // the turn direction should always be consistent with the input parameter.
        double turnDirection = degrees > 0 ? -1:1;
        imu.resetYaw();
        timer.reset();
        double YAW_ERROR_THRESH = 0.2;
        while (Math.abs(error) > YAW_ERROR_THRESH && timer.seconds() < 4 && opMode.opModeIsActive()) {
            // calculate the error , regardless of the target or current turn angle
            error = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) - Math.abs(degrees);
            power = (error * IMUKp) + IMUKd;

            // note: power positive means turn right,
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            power * turnDirection
                    )
            );
        }
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }
}
