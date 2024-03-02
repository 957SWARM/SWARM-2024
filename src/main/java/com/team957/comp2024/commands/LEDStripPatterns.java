package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.LEDConstants;
import com.team957.comp2024.subsystems.led.LED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class LEDStripPatterns {
    int frame = 0;
    Timer timer = new Timer();

    public static LEDStripPatterns instance = new LEDStripPatterns();

    private LEDStripPatterns() {
        timer.restart();
    }

    public Command getBlankPatternCommand(LED led, int start, int length) {
        return fillCommand(led, start, length, 0, 0, 0);
    }

    public Command fillCommand(LED led, int start, int length, int r, int g, int b) {
        return led.run(
                        () -> {
                            for (int i = start; i < start + length; i++) {
                                led.setPixel(i - 1, r, g, b);
                            }
                        })
                .ignoringDisable(true);
    }

    public Command rslBlinkAnimation(LED led, int start, int length, int r, int g, int b) {
        return led.run(
                        () -> {
                            boolean rslState = RobotController.getRSLState();
                            if (rslState) {
                                for (int i = start; i < start + length; i++) {
                                    led.setPixel(i - 1, r, g, b);
                                }
                            } else {
                                for (int i = start; i < start + length; i++) {
                                    led.setPixel(i - 1, 0, 0, 0);
                                }
                            }
                        })
                .ignoringDisable(true);
    }

    public Command movingForwardAnimation(
            LED led,
            int start,
            int length,
            double frameTime,
            boolean isInverted,
            int r,
            int g,
            int b) {
        return led.run(
                        () -> {
                            int currentFrame = frame;
                            if (isInverted) currentFrame = 2 - currentFrame;

                            for (int i = start; i < start + length; i++) {
                                if ((i - 1) % 3 == currentFrame) {
                                    led.setPixel(i, r, g, b);
                                } else {
                                    led.setPixel(i, 0, 0, 0);
                                }
                            }

                            if (frameTime <= timer.get()) {
                                timer.reset();
                                if (frame == 2) {
                                    frame = 0;
                                } else {
                                    frame++;
                                }
                            }
                        })
                .ignoringDisable(true);
    }

    public Command movingForwardBlocksAnimation(
            LED led,
            int start,
            int length,
            double frameTime,
            boolean isInverted,
            int r,
            int g,
            int b) {
        return led.run(
                        () -> {
                            int currentFrame = frame;
                            if (isInverted) currentFrame = 2 - currentFrame;

                            for (int i = start; i < start + length; i++) {
                                if ((i - 1) % 2 == currentFrame || (i - 1) % 3 == currentFrame) {
                                    led.setPixel(i, r, g, b);
                                } else {
                                    led.setPixel(i, 0, 0, 0);
                                }
                            }

                            if (frameTime <= timer.get()) {
                                timer.reset();
                                if (frame == 2) {
                                    frame = 0;
                                } else {
                                    frame++;
                                }
                            }
                        })
                .ignoringDisable(true);
    }

    public Command movingForwardChainAnimation(
            LED led,
            int start,
            int length,
            double frameTime,
            boolean isInverted,
            int r,
            int g,
            int b) {
        return led.run(
                        () -> {
                            int currentFrame = frame;
                            if (isInverted) currentFrame = 2 - currentFrame;

                            boolean on = (currentFrame == 0);

                            for (int i = start; i < start + length; i += 2) {

                                if (on) {
                                    led.setPixel(i, r, g, b);
                                    if (i + 1 < start + length) {
                                        led.setPixel(i + 1, r, g, b);
                                    }
                                } else {
                                    led.setPixel(i, 255, 255, 255);
                                    if (i + 1 < start + length) {
                                        led.setPixel(i + 1, 255, 255, 255);
                                    }
                                }

                                on = !on;
                            }

                            if (frameTime <= timer.get()) {
                                timer.reset();
                                if (frame > 0) {
                                    frame = 0;
                                } else {
                                    frame++;
                                }
                            }
                        })
                .ignoringDisable(true);
    }

    public Command yellowBoltLightCommand(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        return movingForwardAnimation(led, start, length, frameTime, isInverted, 254, 150, 0);
    }

    public Command fullYellowCommand(LED led, int start, int length) {
        return fillCommand(led, start, length, 254, 150, 0);
    }

    public Command allianceColor(LED led, int start, int length, Supplier<Alliance> alliance) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return fillCommand(led, start, length, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return fillCommand(led, start, length, 23, 29, 79);
        } // TODO
    }

    public Command boltAllianceColor(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return movingForwardAnimation(
                    led, start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return movingForwardAnimation(led, start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command blockAllianceColor(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return movingForwardBlocksAnimation(
                    led, start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return movingForwardBlocksAnimation(
                    led, start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command noteInRobotCommand(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(
                led, start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command notePickupCommand(LED led, int start, int length) {
        return fillCommand(led, start, length, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command noteShootingCommand(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        return movingForwardAnimation(
                led, start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command fullGreenCommand(LED led, int start, int length) {
        return fillCommand(led, start, length, 0, LEDConstants.FULL_GREEN_RGB, 0);
    }

    public Command blockGreenCommand(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(
                led, start, length, frameTime, isInverted, 0, LEDConstants.FULL_GREEN_RGB, 0);
    }

    public Command endGameCommand(
            LED led, int start, int length, double frameTime, boolean isInverted) {
        return movingForwardChainAnimation(
                led,
                0,
                LEDConstants.TOTAL_PIXELS,
                frameTime,
                isInverted,
                LEDConstants.FULL_RED_RGB,
                0,
                0);
    }
}
