package com.team957.comp2024;

import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.commands.ActiveNoteCentering;
import com.team957.comp2024.commands.Autos;
import com.team957.comp2024.commands.LEDStripPatterns;
import com.team957.comp2024.commands.NoteTargeting;
import com.team957.comp2024.commands.ScoringSequences;
import com.team957.comp2024.commands.TrajectoryFollowing;
import com.team957.comp2024.commands.VisionAlignment;
import com.team957.comp2024.input.DefaultDriver;
import com.team957.comp2024.input.DriverInput;
import com.team957.comp2024.input.SimKeyboardDriver;
import com.team957.comp2024.peripherals.IMU;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.peripherals.PDH;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.util.LimelightLib;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ConcurrentModificationException;
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
    private final TrajectoryFollowing trajectoryFollowing = TrajectoryFollowing.instance;

    private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    private final Swerve swerve = Swerve.getSwerve(isReal(), isCompetitionRobot());

    private final Shooter shooter = Shooter.getShooter(isReal());

    private final IntakePivot pivot = IntakePivot.getIntakePivot(isReal());

    private final IntakeRoller intakeRoller = IntakeRoller.getIntakeRoller(isReal());

    private final LEDStripPatterns led = new LEDStripPatterns();

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private DriverInput input;

    private Trigger resetFieldRelZero;
    private Trigger noteTracking;

    private Trigger speakerSequence;
    private Trigger intakeSequence;

    private Trigger intakePivotAmp;
    private Trigger intakePivotStow;

    private Trigger shootAmp;

    private Trigger intakeSlow;
    private Trigger outakeSlow;

    private Trigger onGetNote;

    private Trigger ledEndGame;
    private Trigger ledNotePickup;

    private Trigger activeNoteCentering;

    private double fieldRelRotationOffset = 0;

    private final LLlocalization poseEstimation =
            new LLlocalization(
                    SwerveConstants.KINEMATICS,
                    swerve::getStates,
                    swerve::getPositions,
                    imu::getCorrectedAngle,
                    () -> fieldRelRotationOffset,
                    isReal());

    private VisionAlignment visionAlignment;

    private NoteTargeting noteTargeting;

    private final Autos autos =
            new Autos(swerve, pivot, intakeRoller, shooter, poseEstimation, this::getAlliance);

    private final Notifier fastLoop = new Notifier(this::loop);

    private final Alert loopOverrun = new Alert("Loop overrun!", AlertType.WARNING);
    private final Alert canUtil = new Alert("High CAN utilization!", AlertType.WARNING);
    private final Alert practiceBot =
            new Alert("Using practice robot constants!", AlertType.WARNING);
    private final Alert loopSkipped =
            new Alert(
                    "Handling concurrency error: CommandScheduler loop skipped!", AlertType.ERROR);

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

        ui.addAuto("Narrow Center Four Piece", autos.narrowFourPiece());
        ui.addAuto("Source Far Three Piece", autos.sourceFarThreePiece());
        ui.addAuto("Amp Three Piece", autos.ampThreePiece());
        ui.addAuto("Center Three Piece", autos.centerThreePiece());
        ui.addAuto("Center Two Piece", autos.centerTwoPiece());
        // ui.addAuto("Center Four Piece", autos.centerFourPiece());
        // ui.addAuto("Source Two Piece", autos.sourceTwoPiece());
        ui.addAuto("Just Leave: Center", autos.justLeaveCenter());
        ui.addAuto("Just Leave: Amp", autos.justLeaveAmp());
        ui.addAuto("Just Leave: Source", autos.justLeaveSource());
        ui.addAuto("Just Shoot: Center", autos.justShootCenter());
        ui.addAuto("Just Shoot: Amp", autos.justShootAmp());
        ui.addAuto("Just Shoot: Source", autos.justShootSource());

        // ui.addAuto("Five Piece??", autos.fivePieceMockup());
        // ui.addAuto("Test Path", autos.testPath());
        // ui.addAuto("Five Piece Mockup", autos.fivePieceMockup());
        // ui.addAuto("Four Piece Mockup", autos.fourPieceMockup());
        // ui.addAuto("Three Piece Mockup", autos.threePieceMockup());

        noteTargeting =
                new NoteTargeting(
                        swerve,
                        () -> input.swerveX(),
                        () -> input.swerveY(),
                        () -> fieldRelRotationOffset,
                        poseEstimation,
                        VisionConstants.LL1_NAME);

        visionAlignment =
                new VisionAlignment(swerve, input::swerveX, input::swerveY, poseEstimation);

        swerve.setDefaultCommand(
                swerve.getFieldRelativeControlCommand(
                        () ->
                                new ChassisSpeeds(
                                        input.swerveX(), input.swerveY(), input.swerveRot()),
                        () -> poseEstimation.getRotationEstimate()));

        pivot.setDefaultCommand(pivot.toStow());
        shooter.setDefaultCommand(shooter.idle());
        intakeRoller.setDefaultCommand(intakeRoller.idle());
        led.scheduleDefaultCommand(led.allianceColor(0, 50));

        speakerSequence = new Trigger(input::speakerSequence);
        speakerSequence.toggleOnTrue(
                ScoringSequences.coordinatedSubwooferShot(shooter, pivot, intakeRoller));

        intakeSequence = new Trigger(input::intakeSequence);
        intakeSequence.toggleOnTrue(ScoringSequences.coordinatedFloorIntake(pivot, intakeRoller));

        intakePivotStow = new Trigger(input::intakePivotStow);
        intakePivotStow.toggleOnTrue(pivot.toStow());

        intakePivotAmp = new Trigger(input::pivotAmp);
        intakePivotAmp.onTrue(
                TrajectoryFollowing.instance
                        .driveToRelativePose(
                                swerve,
                                poseEstimation::getPoseEstimate,
                                () -> new Transform2d(-.09, 0, new Rotation2d()))
                        .alongWith(
                                pivot.toAmp()
                                        .alongWith(
                                                new WaitCommand(1).andThen(intakeRoller.ampShot())))
                        .withTimeout(2));

        shootAmp = new Trigger(input::shootAmp);
        // shootAmp.onTrue(intakeRoller.ampShotUntilNoteGone());

        shootAmp.whileTrue(
                TrajectoryFollowing.instance.driveToRelativePose(
                        swerve,
                        poseEstimation::getPoseEstimate,
                        () ->
                                new Pose2d(1, 7.5, new Rotation2d(Math.PI / 2))
                                        .minus(poseEstimation.getPoseEstimate())));

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

        resetFieldRelZero = new Trigger(input::zeroGyro);

        resetFieldRelZero.onTrue(
                Commands.runOnce(
                        () -> {
                            //     fieldRelRotationOffset =
                            //             poseEstimation.getRotationEstimate().getRadians();
                            poseEstimation.centerGyro();
                        }));

        ledEndGame = new Trigger(() -> DriverStation.getMatchTime() <= 20);
        ledEndGame.whileTrue(led.endGameCommand(0, 50, .100, false));

        ledNotePickup = new Trigger(() -> intakeRoller.debouncedNoteIsPresent());
        ledNotePickup
                .onTrue(
                        led.notePickupCommand(0, 50)
                                .withTimeout(2)
                                .andThen(led.noteInRobotCommand(0, 50, .1, isAutonomous())))
                .onFalse(led.allianceColor(0, 50));

        activeNoteCentering = new Trigger(input::activeNoteCentering);
        activeNoteCentering.onTrue(new ActiveNoteCentering(intakeRoller).withTimeout(2));

        fastLoop.startPeriodic(MiscConstants.NOMINAL_LOOP_TIME_SECONDS);

        poseEstimation.setPose(new Pose2d(0, 0, new Rotation2d()));
    }

    public void loop() {
        try {
            CommandScheduler.getInstance().run();
            loopSkipped.set(false);
        } catch (ConcurrentModificationException e) {
            loopSkipped.set(true);
        }
        // catch (IndexOutOfBoundsException e){
        // CommandScheduler.getInstance().cancelAll();
        // }

        // something deep in wpilib internals

        double dtSeconds = dt.getTimeSecondsSinceLastCall();

        log("loopTimeSeconds", dtSeconds);

        double canUtilization = RobotController.getCANStatus().percentBusUtilization;

        log("canUtilization", canUtilization);

        loopOverrun.set(dtSeconds > MiscConstants.LOOP_WATCHDOG_TRIGGER_SECONDS);
        canUtil.set(canUtilization > MiscConstants.HIGH_CAN_UTIL_THRESHOLD);
        practiceBot.set(!isCompetitionRobot());

        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();

        poseEstimation.update(isTeleop());

        imu.periodic();
        pdh.periodic();

        // System.out.println(visionAlignment.getSpeakerAngle());
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        poseEstimation.setPose(new Pose2d(0, 0, new Rotation2d()));
        // led.endGameCommand(0, 50, .100, false).schedule();
        // led.scheduleDefaultCommand(led.allianceColor(0, 0));
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();

        swerve.lockDrivetrain().ignoringDisable(true);
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
