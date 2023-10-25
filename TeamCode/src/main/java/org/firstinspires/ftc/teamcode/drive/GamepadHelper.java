package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadHelper {

    public enum GamePadState {
        POSITIVE,
        NEUTRAL,
    }

    private final double  minMultiplier = 0.25;
    private final double  maxMultiplier = 1.00;
    private final double incrementMultiplier = 0.2;
    private double gameStickMultiplier;
    private final double timeIncrementInMs = 200;

    ElapsedTime xGamePadTimer;
    boolean isRamping;
    GamePadState previousGameStickState ;
    GamePadState currentGameStickState ;

    public void init() {
        previousGameStickState = GamePadState.NEUTRAL;
        currentGameStickState = GamePadState.NEUTRAL;
        gameStickMultiplier = minMultiplier;
        xGamePadTimer = new ElapsedTime();
        isRamping = false;
    }

    public double getRampingValue(float gameStick) {
        previousGameStickState = currentGameStickState;

        if (gameStick < 0) {
            currentGameStickState = GamePadState.NEUTRAL;
        } else  if (gameStick > 0) {
            currentGameStickState = GamePadState.POSITIVE;
        } else {
            currentGameStickState = GamePadState.NEUTRAL;
            gameStickMultiplier = minMultiplier;
        }
        if (previousGameStickState != currentGameStickState ){
            isRamping = true;
            gameStickMultiplier = minMultiplier;
            xGamePadTimer.reset();
        }

        if (isRamping && xGamePadTimer.milliseconds() > timeIncrementInMs) {
            if (gameStickMultiplier <= maxMultiplier) {
                gameStickMultiplier += incrementMultiplier;
            } else {
                isRamping = false;
            }

            if (gameStickMultiplier > maxMultiplier) {
                gameStickMultiplier = maxMultiplier;
            }

            xGamePadTimer.reset();
        }

        return gameStick*gameStickMultiplier;
    }
}
