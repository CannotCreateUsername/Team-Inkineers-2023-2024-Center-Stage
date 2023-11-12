package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EndgameSubsystems {
    private enum DroneState {
        READY,
        LAUNCHED
    }

    DroneState droneState;

    private final Servo drone;

    public EndgameSubsystems(HardwareMap hardwareMap) {
        drone = hardwareMap.get(Servo.class, "drone");
    }

    public void run(GamepadEx gamepad) {

        gamepad.readButtons();
    }
}
