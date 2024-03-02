package com.team957.comp2024;

import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.commands.Autos;
import com.team957.comp2024.commands.ChoreoFollowingFactory;
import com.team957.comp2024.commands.LEDStripPatterns;
import com.team957.comp2024.commands.NoteTargeting;
import com.team957.comp2024.commands.OnTheFlyPathing;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
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

    // private final Pivot intakePivot = new Pivot();

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

    private final OnTheFlyPathing otf = OnTheFlyPathing.instance;

    private final Autos autos =
            new Autos(swerve, pivot, intakeRoller, shooter, poseEstimation, this::getAlliance);

    private DriverInput input;

    private final NoteTargeting noteTargeting =
            new NoteTargeting(swerve, poseEstimation, "limelight");

    private final Command noteTrackCommand =
            noteTargeting.getNoteTrackCommand(
                    () -> input.swerveX(), () -> input.swerveY(), () -> input.swerveRot());

    // triggers
    /*

    private Trigger speaker;
    private Trigger amp;
    private Trigger floorIntake;

    private Trigger noteTracking;
    private Trigger otfAmp;
    private Trigger otfSpeaker;
    */
    // temporary trigger for testing:
    private Trigger resetFieldRelZero;
    private Trigger shoot;

    private Trigger intakeStow;
    private Trigger intakeAmp;
    private Trigger intakeFloor;

    private Trigger intakeActive;
    private Trigger intakeEject;

    private Trigger intakeSlowActive;
    private Trigger intakeSlowEject;

    private Trigger grab;
    private Trigger raiseHook;
    private Trigger lowerHook;
    private Trigger climb;

    private Trigger noteTracking;
    private Trigger otfAmp;
    private Trigger otfSpeaker;

    private Trigger onGetNote;

    private double fieldRelRotationOffset = 0;

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

            // CameraServer.startAutomaticCapture().setResolution(480, 360);
        } else {
            input = new SimKeyboardDriver();
        }

        LimelightLib.setPipelineIndex("limelight", 4);

        Monologue.setupMonologue(this, "Robot", false, true);

        DriverStation.startDataLog(DataLogManager.getLog()); // same log used by monologue

        ui.setDriverInputChangeCallback((driverInput) -> this.input = driverInput);
        ui.setOperatorInputChangeCallback((operatorInput) -> {});

        ui.addAuto("testPath", autos.testPath());
        ui.addAuto("fivePieceMockup", autos.fivePieceMockup());
        ui.addAuto("fourPieceMockup", autos.fourPieceMockup());
        ui.addAuto("threePieceMockup", autos.threePieceMockup());

        swerve.setDefaultCommand(
                swerve.getFieldRelativeControlCommand(
                        () ->
                                new ChassisSpeeds(
                                        input.swerveX(), input.swerveY(), input.swerveRot()),
                        () ->
                                poseEstimation
                                        .getRotationEstimate()
                                        .minus(new Rotation2d(fieldRelRotationOffset))));

        /*
        intakePivot.setDefaultCommand(intakePivot.holdStow());
        intakeRoller.setDefaultCommand(intakeRoller.idle());
        */
        // Testing shooter with no voltage
        shooter.setDefaultCommand(shooter.idle());
        intakeRoller.setDefaultCommand(intakeRoller.idle());
        shoot = new Trigger(() -> input.noteTracking());
        shoot.whileTrue(shooter.halfCourtShot());

        intakeAmp = new Trigger(() -> input.speaker());
        intakeAmp.toggleOnTrue(
                ScoringSequences.coordinatedSubwooferShot(shooter, pivot, intakeRoller));

        intakeFloor = new Trigger(() -> input.intakeFloor());
        intakeFloor.toggleOnTrue(ScoringSequences.coordinatedFloorIntake(pivot, intakeRoller));

        intakeStow = new Trigger(() -> input.intakeStow());
        intakeStow.toggleOnTrue(pivot.toStow());

        intakeActive = new Trigger(() -> input.lowerHook());
        intakeActive.whileTrue(intakeRoller.floorIntakeUntilNote());

        intakeEject = new Trigger(() -> input.raiseHook());
        intakeEject.whileTrue(intakeRoller.shooterHandoffUntilNoteGone());

        intakeSlowActive = new Trigger(input::slowIntake);
        intakeSlowActive.whileTrue(intakeRoller.slowIntake());

        onGetNote = new Trigger(intakeRoller::debouncedNoteIsPresent);
        onGetNote.toggleOnTrue(
                Commands.run(() -> input.setRumble(true))
                        .withTimeout(.5)
                        .andThen(Commands.run(() -> input.setRumble(false))));

        onGetNote.onFalse(Commands.run(() -> input.setRumble(false)).ignoringDisable(true));

        noteTracking = new Trigger(() -> input.noteTracking());
        noteTargeting.getNoteTrackCommand(
                () -> input.swerveX(), () -> input.swerveY(), () -> input.swerveRot());

        // intakeSlowActive = new Trigger(() -> input.slowIntake());
        // intakeSlowActive.whileTrue(intakeRoller.)
        // intakeSlowEject = new Trigger(() -> input.slowEject());

        /*
        boxClimber.setDefaultCommand(boxClimber.idleCommand());

        raiseHook = new Trigger(input::raiseHook);
        raiseHook.onTrue(boxClimber.raiseCommand());

        lowerHook = new Trigger(input::lowerHook);
        lowerHook.onTrue(boxClimber.lowerCommand());

        winch.setDefaultCommand(winch.idleCommand());
        climb = new Trigger(input::climb);
        climb.onTrue(winch.raiseCommand());
        */

        resetFieldRelZero = new Trigger(input::zeroGyro);
        /*
        speaker = new Trigger(input::speaker);
        amp = new Trigger(input::amp);
        floorIntake = new Trigger(input::floorIntake);

        noteTracking = new Trigger(input::noteTracking);
        otfAmp = new Trigger(input::otfAmp);
        otfSpeaker = new Trigger(input::otfSpeaker);
        */
        resetFieldRelZero.onTrue(
                Commands.runOnce(
                        () -> {
                            fieldRelRotationOffset =
                                    poseEstimation.getRotationEstimate().getRadians();
                        }));
        /*
        speaker.toggleOnTrue(
                ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                        .andThen(intakePivot.holdPosition()));

        amp.toggleOnTrue(
                ScoringSequences.coordinatedAmpShot(intakePivot, intakeRoller)
                        .andThen(intakePivot.holdPosition()));

        floorIntake.toggleOnTrue(
                ScoringSequences.coordinatedFloorIntake(intakePivot, intakeRoller)
                        .andThen(intakePivot.holdPosition()));

        noteTracking.toggleOnTrue(
                noteTargeting.getNoteTrackCommand(
                        () -> input.swerveX(), () -> input.swerveY(), () -> input.swerveRot()));

        otfAmp.toggleOnTrue(
                otf.otfPathingCommand(
                        swerve,
                        () -> {
                            return (getAlliance() == Alliance.Blue)
                                    ? OtfPathingConstants.OTF_AMP_POSE_BLUE
                                    : OtfPathingConstants.OTF_AMP_POSE_RED;
                        },
                        poseEstimation::getPoseEstimate));

        otfSpeaker.toggleOnTrue(
                otf.otfPathingCommand(
                        swerve,
                        () -> {
                            Alliance alliance =
                                    DriverStation.getAlliance().isPresent()
                                            ? DriverStation.getAlliance().get()
                                            : MiscConstants.DEFAULT_ALLIANCE;

                            return (alliance == Alliance.Blue)
                                    ? OtfPathingConstants.OTF_SPEAKER_POSE_BLUE
                                    : OtfPathingConstants.OTF_SPEAKER_POSE_RED;
                        },
                        poseEstimation::getPoseEstimate));
        */
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
        // shooter.setDefaultCommand(shooter.idle());

        // Default commands
        // boxClimber.setDefaultCommand(boxClimber.idleCommand());
        // winch.setDefaultCommand(winch.idleCommand());
        // intakeRoller.setDefaultCommand(intakeRoller.idleCommand());
        // autoLoadFail.set(false);

        // 31 Pixels on the top of comp bot!!!
        // led.endGameCommand(0, 957, .100, false).schedule();
        led.endGameCommand(0, 50, .100, false).schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        // ui.getAuto().schedule();
        otf.otfPathingCommand(
                        swerve,
                        () -> new Pose2d(1.35, 5.5, new Rotation2d()),
                        poseEstimation::getPoseEstimate)
                .schedule();
    }

    @Log
    public Alliance getAlliance() {
        return DriverStation.getAlliance().isPresent()
                ? DriverStation.getAlliance().get()
                : MiscConstants.DEFAULT_ALLIANCE;
    }
}
