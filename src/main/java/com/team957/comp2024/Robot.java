package com.team957.comp2024;

import com.team957.comp2024.subsystems.IMU;
import com.team957.comp2024.subsystems.PDH;
import com.team957.comp2024.subsystems.PneumaticsHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {
    private final IMU imu = new IMU();
    private final PDH pdh = new PDH();
    private final PneumaticsHub ph = new PneumaticsHub();

    @Override
    public void robotInit() {
        Monologue.setupLogging(this, "/robot");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Monologue.update();
    }
}
