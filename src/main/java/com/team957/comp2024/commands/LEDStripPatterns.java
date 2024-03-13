package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.LEDConstants;
import com.team957.comp2024.subsystems.led.LED;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class LEDStripPatterns {

    int frame = 0;
    Timer timer = new Timer();

    public LEDStripPatterns() {
        timer.restart();
    }

    public Command getBlankPatternCommand(LED led, int start, int length) {
        return led.run(
                () -> {
                    for (int i = start; i < start + length; i++) {
                        led.setPixel(i - 1, 0, 0, 0);
                    }
                });
    }

    public Command getBlankPatternCommand(LED led) {
        return getBlankPatternCommand(led, 0, led.size());
    }

    public Command rslSyncAnimation(
            LED led, IntSupplier r, IntSupplier g, IntSupplier b, int start, int length) {
        return led.run(
                () -> {
                    boolean rslState = RobotController.getRSLState();
                    if (rslState) {
                        for (int i = start; i < start + length; i++) {
                            led.setPixel(i - 1, r.getAsInt(), g.getAsInt(), b.getAsInt());
                        }
                    } else {
                        for (int i = start; i < start + length; i++) {
                            led.setPixel(i - 1, 0, 0, 0);
                        }
                    }
                });
    }

    public Command rslSyncCommand(LED led, IntSupplier r, IntSupplier g, IntSupplier b) {
        return rslSyncAnimation(led, r, g, b, 0, led.size());
    }

    public Command rslSyncAnimation(LED led, IntSupplier r, IntSupplier g, IntSupplier b) {
        return rslSyncAnimation(led, r, g, b, 0, led.size());
    }

    public Command movingForwardAnimation(
            LED led,
            double frameTime,
            boolean isInverted,
            IntSupplier r,
            IntSupplier g,
            IntSupplier b,
            int start,
            int length) {

        return led.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    for (int i = start; i < start + length; i++) {
                        if ((i - 1) % 3 == currentFrame) {
                            led.setPixel(i, r.getAsInt(), g.getAsInt(), b.getAsInt());
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
                });
    }

    public Command movingForwardAnimation(
            LED led,
            double frameTime,
            boolean isInverted,
            IntSupplier r,
            IntSupplier g,
            IntSupplier b) {
        return movingForwardAnimation(led, frameTime, isInverted, r, g, b, 0, led.size());
    }

    public Command movingForwardBlocksAnimation(
            LED led,
            double frameTime,
            boolean isInverted,
            IntSupplier r,
            IntSupplier g,
            IntSupplier b,
            int start,
            int length) {

        return led.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    for (int i = start; i < start + length; i++) {
                        if ((i - 1) % 2 == currentFrame || (i - 1) % 3 == currentFrame) {
                            led.setPixel(i, r.getAsInt(), g.getAsInt(), b.getAsInt());
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
                });
    }

    public Command movingForwardBlocksAnimation(
            LED led,
            double frameTime,
            boolean isInverted,
            IntSupplier r,
            IntSupplier g,
            IntSupplier b) {
        return movingForwardAnimation(led, frameTime, isInverted, r, g, b, 0, led.size());
    }

    public Command movingForwardChainAnimation(
            LED led,
            double frameTime,
            boolean isInverted,
            IntSupplier r,
            IntSupplier g,
            IntSupplier b,
            int start,
            int length) {

        return led.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    boolean on = (currentFrame == 0);

                    for (int i = start; i < start + length; i += 2) {

                        if (on) {
                            led.setPixel(i, r.getAsInt(), g.getAsInt(), b.getAsInt());
                            if (i + 1 < start + length) {
                                led.setPixel(i + 1, r.getAsInt(), g.getAsInt(), b.getAsInt());
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
                });
    }

    public Command movingForwardChainAnimation(
            LED led,
            double frameTime,
            boolean isInverted,
            IntSupplier r,
            IntSupplier g,
            IntSupplier b) {
        return movingForwardChainAnimation(led, frameTime, isInverted, r, g, b, 0, led.size());
    }

    public Command yellowBoltLightCommand(LED led, double frameTime, boolean isInverted) {
        return movingForwardAnimation(
                led, frameTime, isInverted, () -> 254, () -> 150, () -> 0, 0, led.size());
    }

    public Command fullYellowCommand(LED led) {
        return rslSyncAnimation(led, () -> 254, () -> 150, () -> 0).ignoringDisable(true);
    }

    public Command allianceColor(LED led, Supplier<Alliance> alliance) {
        return rslSyncCommand(
                led,
                () -> (alliance.get() == Alliance.Blue) ? 23 : LEDConstants.FULL_RED_RGB,
                () -> (alliance.get() == Alliance.Blue) ? 29 : 0,
                () -> (alliance.get() == Alliance.Blue) ? 79 : 0);
    }

    public Command boltAllianceColor(LED led, double frameTime, boolean isInverted) {
        return movingForwardAnimation(led, frameTime, isInverted, null, null, null);
    }

    public Command blockAllianceColor(LED led, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(led, frameTime, isInverted, null, null, null);
    }

    public Command noteInRobotCommand(LED led, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(
                led, frameTime, isInverted, () -> LEDConstants.FULL_RED_RGB, () -> 40, () -> 0);
    }

    public Command notePickupCommand(LED led) {
        return rslSyncAnimation(led, () -> LEDConstants.FULL_RED_RGB, () -> 40, () -> 0);
    }

    public Command noteShootingCommand(LED led, double frameTime, boolean isInverted) {
        return movingForwardAnimation(
                led, frameTime, isInverted, () -> LEDConstants.FULL_RED_RGB, () -> 40, () -> 0);
    }

    public Command fullGreenCommand(LED led) {
        return rslSyncAnimation(led, () -> 0, () -> LEDConstants.FULL_GREEN_RGB, () -> 0);
    }

    public Command blockGreenCommand(LED led, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(
                led, frameTime, isInverted, () -> 0, () -> LEDConstants.FULL_GREEN_RGB, () -> 0);
    }

    public Command endGameCommand(LED led, double frameTime, boolean isInverted) {
        return movingForwardChainAnimation(
                led, frameTime, isInverted, () -> LEDConstants.FULL_RED_RGB, () -> 0, () -> 0);
    }
}
