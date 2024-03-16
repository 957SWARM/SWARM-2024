package com.team957.comp2024;

import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.commands.Autos;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.commands.LEDStripPatterns;
import com.team957.comp2024.commands.NoteTargeting;
import com.team957.comp2024.commands.ScoringSequences;
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
import com.team957.comp2024.util.LimelightLib;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot implements Logged {

    private final DigitalInput practiceBotJumper =
            new DigitalInput(MiscConstants.PRACTICE_BOT_JUMPER_CHANNEL);

    // these need to be constructed/declared so that
    // monologue can work its reflection magic
    private final IMU imu = new IMU();

    private final UI ui = UI.instance;

    @SuppressWarnings("unused")
    private final ChoreoFollowingFactory trajectoryFollowing = ChoreoFollowingFactory.instance;

    private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    private final Swerve swerve = Swerve.getSwerve(isReal(), isCompetitionRobot());

    private final Shooter shooter = Shooter.getShooter(isReal());

    private final IntakePivot pivot = IntakePivot.getIntakePivot(isReal());

    private final IntakeRoller intakeRoller = IntakeRoller.getIntakeRoller(isReal());

    private final BoxClimber boxClimber = BoxClimber.getBoxClimber(isReal());

    private final Winch winch = Winch.getWinch(isReal());

    private final LEDStripPatterns led = new LEDStripPatterns();

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private final LLlocalization poseEstimation =
            new LLlocalization(
                    SwerveConstants.KINEMATICS,
                    swerve::getStates,
                    swerve::getPositions,
                    imu::getCorrectedAngle,
                    isReal());

    private final Autos autos =
            new Autos(swerve, pivot, intakeRoller, shooter, poseEstimation, this::getAlliance);

    private DriverInput input;

    private Trigger resetFieldRelZero;
    private Trigger noteTracking;

    private Trigger speakerSequence;
    private Trigger intakeSequence;

    private Trigger intakePivotAmp;
    private Trigger intakePivotStow;

    private Trigger shootAmp;

    private Trigger climbHookDown;
    private Trigger climbHookUp;
    private Trigger climbWinch;

    private Trigger intakeSlow;
    private Trigger outakeSlow;

    private Trigger onGetNote;

    private double fieldRelRotationOffset = 0;

    private final NoteTargeting noteTargeting =
            new NoteTargeting(
                    swerve,
                    () -> input.swerveX(),
                    () -> input.swerveY(),
                    () -> fieldRelRotationOffset,
                    poseEstimation,
                    VisionConstants.LL1_NAME);

    private final Notifier fastLoop = new Notifier(this::loop);

    private final Alert loopOverrun = new Alert("Loop overrun!", AlertType.WARNING);
    private final Alert canUtil = new Alert("High CAN utilization!", AlertType.WARNING);
    private final Alert practiceBot =
            new Alert("Using practice robot constants!", AlertType.WARNING);

    @Log
    public boolean isCompetitionRobot() {
        return practiceBotJumper.get();
    }

    @Override
    public void robotInit() {

        CommandScheduler.getInstance()
                .setPeriod(Constants.MiscConstants.LOOP_WATCHDOG_TRIGGER_SECONDS);

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();

        if (isReal()) {
            URCL.start(); // URCL segfaults in sim

            input = new DefaultDriver();

            CameraServer.startAutomaticCapture().setResolution(480, 360);
        } else {
            input = new SimKeyboardDriver();
        }

        LimelightLib.setPipelineIndex(
                VisionConstants.LL1_NAME, VisionConstants.LL1_NOTE_TRACKING_PL);
        LimelightLib.setPipelineIndex(
                VisionConstants.LL2_NAME, VisionConstants.LL2_POSE_ESTIMATION_PL);

        Monologue.setupMonologue(this, "Robot", false, true);

        DriverStation.startDataLog(DataLogManager.getLog()); // same log used by monologue

        ui.setDriverInputChangeCallback((driverInput) -> this.input = driverInput);
        ui.setOperatorInputChangeCallback((operatorInput) -> {});

        ui.addAuto("Just Leave: Amp", autos.justLeaveAmp());
        ui.addAuto("Just Leave: Center", autos.justLeaveCenter());
        ui.addAuto("Just Leave: Source", autos.justLeaveSource());
        ui.addAuto("Center Two Piece", autos.centerTwoPiece());
        ui.addAuto("Center Four Piece", autos.centerFourPiece());
        ui.addAuto("Test Path", autos.testPath());
        ui.addAuto("Five Piece Mockup", autos.fivePieceMockup());
        ui.addAuto("Four Piece Mockup", autos.fourPieceMockup());
        ui.addAuto("Three Piece Mockup", autos.threePieceMockup());

        swerve.setDefaultCommand(
                swerve.getFieldRelativeControlCommand(
                        () ->
                                new ChassisSpeeds(
                                        input.swerveX(), input.swerveY(), input.swerveRot()),
                        () ->
                                poseEstimation
                                        .getRotationEstimate()
                                        .minus(new Rotation2d(fieldRelRotationOffset))));

        pivot.setDefaultCommand(pivot.toStow());
        shooter.setDefaultCommand(shooter.idle());
        intakeRoller.setDefaultCommand(intakeRoller.idle());
        boxClimber.setDefaultCommand(boxClimber.idleCommand());
        winch.setDefaultCommand(winch.idleCommand());

        speakerSequence = new Trigger(input::speakerSequence);
        speakerSequence.toggleOnTrue(
                ScoringSequences.coordinatedSubwooferShot(shooter, pivot, intakeRoller));

        intakeSequence = new Trigger(input::intakeSequence);
        intakeSequence.toggleOnTrue(ScoringSequences.coordinatedFloorIntake(pivot, intakeRoller));

        intakePivotStow = new Trigger(input::intakePivotStow);
        intakePivotStow.toggleOnTrue(pivot.toStow());

        intakePivotAmp = new Trigger(input::pivotAmp);
        intakePivotAmp.onTrue(pivot.toAmp());

        shootAmp = new Trigger(input::shootAmp);
        shootAmp.onTrue(intakeRoller.ampShotUntilNoteGone());

        intakeSlow = new Trigger(input::slowIntake);
        intakeSlow.whileTrue(intakeRoller.slowIntake());

        outakeSlow = new Trigger(input::slowEject);
        outakeSlow.whileTrue(intakeRoller.slowEject());

        onGetNote = new Trigger(intakeRoller::debouncedNoteIsPresent);
        onGetNote.onTrue(
                Commands.run(() -> input.setRumble(true))
                        .withTimeout(.5)
                        .andThen(Commands.run(() -> input.setRumble(false))));

        onGetNote.onFalse(Commands.run(() -> input.setRumble(false)).ignoringDisable(true));

        noteTracking =
                new Trigger(() -> input.noteTracking() && !intakeRoller.debouncedNoteIsPresent());
        noteTracking.whileTrue(noteTargeting.getNoteTrackCommand());

        // climbHookUp = new Trigger(input::raiseHook);
        // climbHookUp.onTrue(boxClimber.raiseCommand());

        // climbHookDown = new Trigger(input::lowerHook);
        // climbHookDown.onTrue(boxClimber.lowerCommand());

        // climbWinch = new Trigger(input::climbWinch);
        // climbWinch.onTrue(winch.raiseCommand());

        resetFieldRelZero = new Trigger(input::zeroGyro);

        resetFieldRelZero.onTrue(
                Commands.runOnce(
                        () -> {
                            fieldRelRotationOffset =
                                    poseEstimation.getRotationEstimate().getRadians();
                        }));

        fastLoop.startPeriodic(MiscConstants.NOMINAL_LOOP_TIME_SECONDS);
    }

    public void loop() {
        CommandScheduler.getInstance().run();

        double dtSeconds = dt.getTimeSecondsSinceLastCall();

        log("loopTimeSeconds", dtSeconds);

        double canUtilization = RobotController.getCANStatus().percentBusUtilization;

        log("canUtilization", canUtilization);

        loopOverrun.set(dtSeconds > MiscConstants.LOOP_WATCHDOG_TRIGGER_SECONDS);
        canUtil.set(canUtilization > MiscConstants.HIGH_CAN_UTIL_THRESHOLD);
        practiceBot.set(!isCompetitionRobot());

        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();

        poseEstimation.update();

        imu.periodic();
        pdh.periodic();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        led.endGameCommand(0, 50, .100, false).schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        ui.getAuto().schedule();
    }

    @Log
    public Alliance getAlliance() {
        return DriverStation.getAlliance().isPresent()
                ? DriverStation.getAlliance().get()
                : MiscConstants.DEFAULT_ALLIANCE;
    }
}
