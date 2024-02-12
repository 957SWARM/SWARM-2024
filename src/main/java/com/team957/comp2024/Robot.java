package com.team957.comp2024;

import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.commands.Autos;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.commands.NoteTargeting;
import com.team957.comp2024.commands.OnTheFlyPathing;
import com.team957.comp2024.input.DefaultDriver;
import com.team957.comp2024.input.DriverInput;
import com.team957.comp2024.input.SimKeyboardDriver;
import com.team957.comp2024.peripherals.IMU;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.peripherals.PDH;
import com.team957.comp2024.subsystems.climbing.BoxClimber;
import com.team957.comp2024.subsystems.climbing.Winch;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private final OnTheFlyPathing otf = OnTheFlyPathing.instance;

    private final Autos autos =
            new Autos(swerve, intakePivot, intakeRoller, shooter, poseEstimation);

    private DriverInput input;

    private final NoteTargeting noteTargeting =
            new NoteTargeting(swerve, poseEstimation, "limelight");

    // triggers
    private Trigger resetFieldRelZero;

    private Trigger speaker;
    private Trigger amp;
    private Trigger floorIntake;

    private Trigger raiseHook;
    private Trigger lowerHook;
    private Trigger climb;

    private Trigger noteTracking;
    private Trigger aprilTagTrackingTrigger;

    private double fieldRelRotationOffset = 0;

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();

        if (isReal()) {
            URCL.start(); // URCL segfaults in sim

            input = new DefaultDriver();
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

        swerve.setDefaultCommand(
                swerve.getFieldRelativeControlCommand(
                        () ->
                                new ChassisSpeeds(
                                        input.swerveX(), input.swerveY(), input.swerveRot()),
                        () ->
                                poseEstimation
                                        .getRotationEstimate()
                                        .minus(new Rotation2d(fieldRelRotationOffset))));

        intakePivot.setDefaultCommand(intakePivot.holdStow());
        intakeRoller.setDefaultCommand(intakeRoller.idle());

        shooter.setDefaultCommand(shooter.idle());

        resetFieldRelZero = new Trigger(input::zeroGyro);

        speaker = new Trigger(input::speaker);
        amp = new Trigger(input::amp);
        floorIntake = new Trigger(input::floorIntake);

        noteTracking = new Trigger(input::noteTracking);
        aprilTagTrackingTrigger = new Trigger(input::enableAprilTagTracking);

        resetFieldRelZero.onTrue(
                Commands.runOnce(
                        () -> {
                            fieldRelRotationOffset =
                                    poseEstimation.getRotationEstimate().getRadians();
                        }));

        speaker.toggleOnTrue(
                shooter.subwooferShot()
                        .raceWith(
                                intakePivot
                                        .toHandoff()
                                        .andThen(
                                                intakeRoller
                                                        .shooterHandoffUntilNoteGone()
                                                        .alongWith(intakePivot.holdPosition())))
                        .andThen(intakePivot.holdStow()));

        amp.toggleOnTrue(
                intakePivot
                        .toAmp()
                        .andThen(
                                intakeRoller
                                        .ampShotUntilNoteGone()
                                        .alongWith(intakePivot.holdPosition()))
                        .andThen(intakePivot.holdStow()));

        floorIntake.toggleOnTrue(
                intakePivot
                        .toFloor()
                        .andThen(
                                intakeRoller
                                        .floorIntakeUntilNote()
                                        .alongWith(intakePivot.holdPosition()))
                        .andThen(intakePivot.holdStow()));

        noteTracking.onTrue(
                noteTargeting.getNoteTrackCommand(
                        () -> input.swerveX(), () -> input.swerveY(), () -> input.swerveRot()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        log("loopTimeSeconds", dt.getTimeSecondsSinceLastCall());

        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();

        poseEstimation.update();

        imu.periodic();
        pdh.periodic();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        // ui.getAuto().schedule();
    }
}
