package com.team957.comp2024;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.input.DefaultDriver;
import com.team957.comp2024.input.DriverInput;
import com.team957.comp2024.input.SimKeyboardDriver;
import com.team957.comp2024.subsystems.IMU;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.util.SwarmChoreo;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot implements Logged {
    // these need to be constructed so that
    // monologue can work its reflection magic
    private final IMU imu = new IMU();

    @SuppressWarnings("unused")
    // private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    private final Swerve swerve = Swerve.getSwerve(isReal());

    private final IntakePivot intakePivot = IntakePivot.getIntakePivot(isReal());

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private final Localization localization =
            new Localization(
                    swerve::getStates, swerve::getPositions, imu::getCorrectedAngle, !isReal());

    // done this way for monologue's sake
    private final ChoreoFollowingFactory trajectoryFollowing = new ChoreoFollowingFactory();

    private final Alert autoLoadFail = new Alert("Auto path failed to load!", AlertType.ERROR);

    private DriverInput input;

    private final Command teleopDrive =
            swerve.getFieldRelativeControlCommand(
                    () -> {
                        return new ChassisSpeeds(
                                input.swerveX(), input.swerveY(), input.swerveRot());
                    },
                    localization::getRotationEstimate);

    private final Command teleopIntake = intakePivot.goToSetpoint(() -> 1.0);

    private Command autoCommand = new InstantCommand();

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();

        if (isReal()) {
            URCL.start(); // URCL segfaults in sim

            input = new DefaultDriver();
            // implement more options later
        } else {
            input = new SimKeyboardDriver();
        }

        Monologue.setupMonologue(this, "Robot", false, true);

        DriverStation.startDataLog(DataLogManager.getLog()); // same log used by monologue
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

        teleopIntake.schedule();

        autoLoadFail.set(false);
    }

    @Override
    public void disabledPeriodic() {
        boolean autoUpdated = true; // todo figure this out

        if (autoUpdated) {
            ChoreoTrajectory traj = SwarmChoreo.getTrajectory("TestPath");

            if (traj == null) {
                autoCommand = new InstantCommand();

                autoLoadFail.set(true);
            } else {
                autoLoadFail.set(false);

                autoCommand =
                        Commands.runOnce(() -> localization.setPose(traj.getInitialPose()))
                                .andThen(
                                        trajectoryFollowing.getPathFollowingCommand(
                                                swerve, traj, localization::getPoseEstimate));
            }
        }
    }

    @Override
    public void autonomousInit() {
        autoCommand.schedule();
    }
}
