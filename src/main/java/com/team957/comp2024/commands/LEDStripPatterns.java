package com.team957.comp2024.commands;

import com.team957.comp2024.Constants;
import com.team957.comp2024.subsystems.led.LED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LEDStripPatterns {

    LED led = new LED(Constants.MiscConstants.TOTAL_PIXELS);
    int frame = 0;
    Timer timer = new Timer();

    public LEDStripPatterns() {
        timer.restart();
    }

    public Command getBlankPatternCommand(int start, int length) {
        return Commands.run(
                () -> {
                    for (int i = start; i < start + length; i++) {
                        led.setPixel(i - 1, 0, 0, 0);
                    }
                });
    }

    public Command yellowBoltLightCommand(int start, int length, double frameTime) {
        return Commands.run(
                () -> {
                    for (int i = start; i < start + length; i++) {
                        if ((i - 1) % 3 == frame) {
                            led.setPixel(i, 255, 255, 0);
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
                },
                led);
    }
}
