package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EndgameSubsystems {

    /** @noinspection FieldCanBeLocal*/ // Drone Launcher Positions
    private final double TAKEOFF = -1;
    /** @noinspection FieldCanBeLocal*/ // Lol die warnings
    private final double LAUNCHED = -1;

    private final Servo droneLauncher;

    public EndgameSubsystems(HardwareMap hardwareMap) {
        droneLauncher = hardwareMap.get(Servo.class, "drone");
        droneLauncher.setPosition(TAKEOFF);
    }

    public void run(GamepadEx gamepad) {
        if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            droneLauncher.setPosition(LAUNCHED);
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            droneLauncher.setPosition(TAKEOFF);
        }
    }

    public String getLauncherState() {
        if (droneLauncher.getPosition() == LAUNCHED) {
            return "LAUNCHED";
        } else {
            return "TAKEOFF";
        }
    }
}
