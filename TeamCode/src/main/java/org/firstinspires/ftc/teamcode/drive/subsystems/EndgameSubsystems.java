package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EndgameSubsystems {

    /** @noinspection FieldCanBeLocal*/ // Drone Launcher Positions
    private final double TAKEOFF = 0.5;
    /** @noinspection FieldCanBeLocal*/ // Lol die warnings
    private final double LAUNCHED = 0.7;

    private final Servo droneLauncher;
    ElapsedTime droneTimer;

    public EndgameSubsystems(HardwareMap hardwareMap) {
        droneLauncher = hardwareMap.get(Servo.class, "drone");
        droneLauncher.setPosition(TAKEOFF);
        droneTimer = new ElapsedTime();
    }

    private boolean launched = false;
    public void run(GamepadEx gamepad1, GamepadEx gamepad2) {
        if ((gamepad1.wasJustReleased(GamepadKeys.Button.Y) || gamepad2.wasJustReleased(GamepadKeys.Button.Y)) && !launched) {
            launchDrone();
            launched = true;
        } else if ((gamepad1.wasJustReleased(GamepadKeys.Button.Y) || gamepad2.wasJustReleased(GamepadKeys.Button.Y)) && launched) {
            droneLauncher.setPosition(TAKEOFF);
            launched = false;
        }
        if (launched && droneTimer.seconds() > 0.2) {

        }
    }

    public String getLauncherState() {
        if (droneLauncher.getPosition() == LAUNCHED) {
            return "LAUNCHED";
        } else if (droneLauncher.getPosition() == TAKEOFF) {
            return "TAKEOFF";
        } else {
            return "WEIRD";
        }
    }

    private void launchDrone() {
        droneTimer.reset();
        droneLauncher.setPosition(TAKEOFF-0.1);
        droneLauncher.setPosition(LAUNCHED);
    }
}
