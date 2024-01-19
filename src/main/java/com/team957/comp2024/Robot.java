package com.team957.comp2024;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.subsystems.IMU;
import com.team957.comp2024.subsystems.PDH;
import com.team957.comp2024.subsystems.PneumaticsHub;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.util.SwarmChoreo;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot implements Logged {
    // these need to be constructed so that
    // monologue can work its reflection magic
    private final IMU imu = new IMU();

    @SuppressWarnings("unused")
    private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    @SuppressWarnings("unused")
    private final PneumaticsHub ph = new PneumaticsHub();

    private final Swerve swerve = Swerve.getSwerve(isReal());

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private final Localization localization =
            new Localization(
                    swerve::getStates, swerve::getPositions, imu::getCorrectedAngle, !isReal());

    // done this way for monologue's sake
    private final ChoreoFollowingFactory trajectoryFollowing = new ChoreoFollowingFactory();

    private final XboxController controller = new XboxController(0);

    public static final UI ui = new UI();

    private final Command teleopDrive =
            swerve.getFieldRelativeControlCommand(
                    () -> {
                        return new ChassisSpeeds(
                                -5 * controller.getLeftY(),
                                -5 * controller.getLeftX(),
                                10 * controller.getLeftTriggerAxis());
                    },
                    localization::getRotationEstimate);

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();

        if (isReal()) URCL.start(); // segfaults in sim

        Monologue.setupMonologue(this, "Robot", false, true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        log("loopTimeSeconds", dt.getTimeSecondsSinceLastCall());

        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();

        localization.update();
    }

    @Override
    public void teleopInit() {
        teleopDrive.schedule();
    }

    @Override
    public void autonomousInit() {
        ChoreoTrajectory traj = SwarmChoreo.getTrajectory("TestPath");

        if (traj != null) {
            localization.setPose(traj.getInitialPose());

            trajectoryFollowing
                    .getPathFollowingCommand(
                            swerve,
                            SwarmChoreo.getTrajectory("TestPath"),
                            localization::getPoseEstimate)
                    .schedule();
        }
    }
}
