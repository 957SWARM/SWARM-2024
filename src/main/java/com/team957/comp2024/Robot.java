package com.team957.comp2024;

import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;
import org.littletonrobotics.urcl.URCL;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.Constants.ShooterConstants;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.input.DefaultDriver;
import com.team957.comp2024.input.DriverInput;
import com.team957.comp2024.input.SimKeyboardDriver;
import com.team957.comp2024.subsystems.IMU;
import com.team957.comp2024.subsystems.PDH;
import com.team957.comp2024.subsystems.climbing.BoxClimber;
import com.team957.comp2024.subsystems.climbing.Winch;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {
    // these need to be constructed so that
    // monologue can work its reflection magic
    private final IMU imu = new IMU();

    @SuppressWarnings("unused")
    private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    private final Swerve swerve = Swerve.getSwerve(isReal());

    private final Shooter shooter = Shooter.getShooter(isReal());

    private final IntakePivot intakePivot = IntakePivot.getIntakePivot(isReal());

    private final IntakeRoller intakeRoller = IntakeRoller.getIntakeRoller(isReal());

    private final BoxClimber boxClimber = BoxClimber.getBoxClimber(isReal());

    private final Winch winch = Winch.getWinch(isReal());

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private final Localization localization =
            new Localization(
                    swerve::getStates, swerve::getPositions, imu::getCorrectedAngle, !isReal());

    // done this way for monologue's sake
    private final ChoreoFollowingFactory trajectoryFollowing = new ChoreoFollowingFactory();

    private final DriverInput driver = new DefaultDriver();

    private final Alert autoLoadFail = new Alert("Auto path failed to load!", AlertType.ERROR);

    private DriverInput input;

    // input.swerveX(), input.swerveY(), input.swerveRot()
    private final Command teleopDrive =
            swerve.getFieldRelativeControlCommand(
                    () -> {
                        return new ChassisSpeeds(
                                input.swerveX(), input.swerveY(), input.swerveRot());
                    },
                    localization::getRotationEstimate);

    // variables
    private double shooterVoltage = 0;
    // whether or not the robot has a note
    private boolean hasNote = false;

    // triggers
    private Trigger shoot;
    private Trigger intake;
    private Trigger puke;
    private Trigger raiseHook;
    private Trigger lowerHook;
    private Trigger climb;


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

        // trigger definitions:
        // shoot trigger needs to also check if intakePivot is retracted
        shoot =
            new Trigger(driver::shoot)
                    .toggleOnTrue(
                            Commands.runOnce(
                                    () -> shooterVoltage = ShooterConstants.DEFAULT_VOLTAGE))
                    .toggleOnTrue(
                        intakeRoller.pukeNoteCommand()  //pukes note into shooter
                    )
                    .toggleOnFalse(
                            Commands.runOnce(
                                    () -> shooterVoltage = ShooterConstants.SHOOTING_VOLTAGE))
                    .toggleOnFalse(
                        intakeRoller.idleCommand()  // makes sure the intake is off
                    );
        
        // should add intakePivot into this trigger so that both the roller and intakePivot coordinate
        intake =
            new Trigger(() -> !hasNote && driver.intake())
                .toggleOnTrue(
                    intakeRoller.intakeNoteCommand()
                )
                .toggleOnFalse(
                    intakeRoller.idleCommand()
                );

        // should add intakePivot into this trigger so that both the roller and intakePivot coordinate
        puke = 
            new Trigger(driver::puke)
                .toggleOnTrue(
                    intakeRoller.pukeNoteCommand()
                )
                .toggleOnFalse(
                    intakeRoller.idleCommand()
                );
                
        raiseHook = 
            new Trigger(driver::raiseHook)
                .onTrue(
                    boxClimber.raiseCommand()
                )
                .onFalse(
                    boxClimber.idleCommand()
                );
        
        lowerHook = 
            new Trigger(driver::lowerHook)
                .onTrue(
                    boxClimber.lowerCommand()
                )
                .onFalse(
                    boxClimber.idleCommand()
                );

        climb =
            new Trigger(driver::climb)
                .onTrue(
                    winch.raiseCommand()
                )
                .onFalse(
                    winch.idleCommand()
                );
                
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
        shooter.defaultShooterControlCommand(() -> shooterVoltage).schedule();

        teleopIntake.schedule();

        // Default commands
        boxClimber.setDefaultCommand(boxClimber.idleCommand());
        winch.setDefaultCommand(winch.idleCommand());
        intakeRoller.setDefaultCommand(intakeRoller.idleCommand());

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
