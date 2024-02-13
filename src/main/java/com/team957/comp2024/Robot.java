package com.team957.comp2024;

import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.commands.Autos;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.commands.NoteTargeting;
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
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot implements Logged {
    // these need to be constructed/declared so that
    // monologue can work its reflection magic
    private final IMU imu = new IMU();

    private final UI ui = UI.instance;

    @SuppressWarnings("unused")
    private final ChoreoFollowingFactory trajectoryFollowing = ChoreoFollowingFactory.instance;

    @SuppressWarnings("unused")
    private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    private final Swerve swerve = Swerve.getSwerve(isReal());

    private final Shooter shooter = Shooter.getShooter(isReal());

    private final IntakePivot intakePivot = IntakePivot.getIntakePivot(isReal());

    private final IntakeRoller intakeRoller = IntakeRoller.getIntakeRoller(isReal());

    private final BoxClimber boxClimber = BoxClimber.getBoxClimber(isReal());

    private final Winch winch = Winch.getWinch(isReal());

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private final LLlocalization poseEstimation =
            new LLlocalization(
                    SwerveConstants.KINEMATICS,
                    swerve::getStates,
                    swerve::getPositions,
                    imu::getCorrectedAngle,
                    isReal());

    private final Autos autos =
            new Autos(swerve, intakePivot, intakeRoller, shooter, poseEstimation);

    private DriverInput input;

    NoteTargeting noteTargeting = new NoteTargeting(swerve, poseEstimation, "limelight");

    private final Command noteTrackCommand =
            noteTargeting.getNoteTrackCommand(
                    () -> input.swerveX(), () -> input.swerveY(), () -> input.swerveRot());

    // done this way for monologue's sake
    // private final ChoreoFollowingFactory trajectoryFollowing = new ChoreoFollowingFactory();

    private final DriverInput driver = new DefaultDriver();

    private final Command teleopDrive =
            swerve.getFieldRelativeControlCommand(
                    () -> {
                        return new ChassisSpeeds(
                                input.swerveX(), input.swerveY(), input.swerveRot());
                    },
                    poseEstimation::getRotationEstimate);

    // whether or not the robot has a note
    private boolean hasNote = false;

    // triggers
    private Trigger shoot;
    private Trigger intake;
    private Trigger eject;
    private Trigger raiseHook;
    private Trigger lowerHook;
    private Trigger climb;
    private Trigger noteTrackingTrigger;
    private Trigger aprilTagTrackingTrigger;
    //  private Trigger pivotAmp;

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

        ui.addAuto("Shoot Preload", autos.shootPreloadBumperAuto());
        ui.addAuto("Middle Two Piece", autos.middleTwoPiece());
        ui.addAuto("Top Near Three Piece", autos.topNearThreePiece());
        ui.addAuto("Top Center Four Piece", autos.topCenterFourPiece());
        ui.addAuto("Near Four Piece", autos.nearFourPiece());
        ui.addAuto("Top Five Piece", autos.topFivePiece());

        ui.setDriverInputChangeCallback((driverInput) -> this.input = driverInput);
        ui.setOperatorInputChangeCallback((operatorInput) -> {});

        // trigger definitions:
        // shoot trigger needs to also check if intakePivot is retracted
        // shoot =
        //         new Trigger(driver::shoot)
        //                 .toggleOnTrue(shooter.idle())
        //                 .toggleOnTrue(
        //                         intakeRoller.ejectNoteCommand() // ejects note into shooter
        //                         )
        //                 .toggleOnFalse(shooter.subwooferShot())
        //                 .toggleOnFalse(
        //                         intakeRoller.idleCommand() // makes sure the intake is off
        //                         );

        // TODO: need to reimplement all pivot/roller/shooter superstructure logic, badly

        // should add intakePivot into this trigger so that both the roller and
        // intakePivot coordinate
        // intake =
        //         new Trigger(() -> !hasNote && driver.intake())
        //                 .toggleOnTrue(intakeRoller.intakeNoteCommand())
        //                 .toggleOnFalse(intakeRoller.idleCommand());

        // // should add intakePivot into this trigger so that both the roller and
        // // intakePivot coordinate
        // eject =
        //         new Trigger(driver::eject)
        //                 .toggleOnTrue(intakeRoller.ejectNoteCommand())
        //                 .toggleOnFalse(intakeRoller.idleCommand());

        // raiseHook =
        //         new Trigger(driver::raiseHook)
        //                 .onTrue(boxClimber.raiseCommand())
        //                 .onFalse(boxClimber.idleCommand());

        // lowerHook =
        //         new Trigger(driver::lowerHook)
        //                 .onTrue(boxClimber.lowerCommand())
        //                 .onFalse(boxClimber.idleCommand());

        // climb =
        //         new Trigger(driver::climb)
        //                 .onTrue(winch.raiseCommand())
        //                 .onFalse(winch.idleCommand());

        // noteTrackingTrigger =
        //         new Trigger(() -> input.enableNoteTracking() && noteTargeting.checkTarget())
        //                 .whileTrue(noteTrackCommand);

        /*       pivotAmp = new Trigger(driver::pivotAmp)
        .onTrue(
                intakeRoller.intakeNoteCommand()
        )
        .onFalse(
                intakeRoller.idleCommand()
        ); */
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        log("loopTimeSeconds", dt.getTimeSecondsSinceLastCall());

        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();

        poseEstimation.update();
    }

    @Override
    public void teleopInit() {

        swerve.setDefaultCommand(teleopDrive);

        // shooter.setDefaultCommand(shooter.idle());

        // Default commands
        // boxClimber.setDefaultCommand(boxClimber.idleCommand());
        // winch.setDefaultCommand(winch.idleCommand());
        // intakeRoller.setDefaultCommand(intakeRoller.idleCommand());
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        // ui.getAuto().schedule();
    }
}
