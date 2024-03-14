package com.team957.comp2024.commands;

import com.team957.comp2024.UI;
import com.team957.comp2024.subsystems.climbing.BoxClimber;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.led.LED;
import com.team957.comp2024.subsystems.shooter.Shooter;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.subsystems.swerve.Swerve.ModuleIO;
import com.team957.lib.math.UtilityMath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SelfTest {
    private SelfTest() {}

    // java is so bad
    // we have to do stuff like this to smuggle state into our functional closures
    private static class BooleanReference {
        boolean bool;

        BooleanReference(boolean bool) {
            this.bool = bool;
        }
    }

    private static record SelfTestCommand(
            Command command,
            String runningMessage,
            String successMessage,
            String failureMessage,
            BooleanSupplier success) {} // value of result is meaningless until command finished

    private static SelfTestCommand testNonzero(
            String name,
            Consumer<Double> setVoltage,
            Supplier<Double> measurement,
            double tolerance,
            double voltage,
            Subsystem... requirements) {
        BooleanReference failed = new BooleanReference(false);
        return new SelfTestCommand(
                Commands.run(() -> setVoltage.accept(voltage), requirements)
                        .raceWith(
                                new WaitCommand(.25)
                                        .andThen(
                                                Commands.runOnce(
                                                        () ->
                                                                failed.bool =
                                                                        Math.abs(measurement.get())
                                                                                        > tolerance
                                                                                || failed.bool)))
                        .andThen(
                                Commands.run(() -> setVoltage.accept(-voltage))
                                        .raceWith(
                                                new WaitCommand(.25)
                                                        .andThen(
                                                                Commands.runOnce(
                                                                        () ->
                                                                                failed.bool =
                                                                                        Math.abs(
                                                                                                                measurement
                                                                                                                        .get())
                                                                                                        > tolerance
                                                                                                || failed.bool))))
                        .finallyDo(() -> setVoltage.accept(0.0)),
                name + " nonzero self test running!",
                name + " nonzero self test passed!",
                name + " nonzero self test failed!",
                () -> failed.bool);
    }

    private static SelfTestCommand testGoToSetpoint(
            String name,
            Consumer<Double> setSetpoint,
            Supplier<Double> measurement,
            double setpoint,
            double settlingTimeSeconds,
            boolean angular,
            Subsystem... requirements) {
        final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(5);
        final double TOLERANCE_PROPORTION = .05; // 2%

        return new SelfTestCommand(
                Commands.run(() -> setSetpoint.accept(setpoint), requirements)
                        .withTimeout(settlingTimeSeconds),
                name + " setpoint self test running!",
                name + " setpoint self test passed!",
                name + " setpoint self test failed!",
                () ->
                        angular
                                ? UtilityMath.smallestAngleRadiansBetween(
                                                setpoint, measurement.get())
                                        < ANGLE_TOLERANCE_RADIANS
                                : UtilityMath.epsilonEqualsProportion(
                                        setpoint, measurement.get(), TOLERANCE_PROPORTION));
    }

    private static Command flashFailure(LED led) {
        LEDStripPatterns patterns = new LEDStripPatterns();

        return patterns.rslSyncAnimation(led, () -> 255, () -> 0, () -> 0).ignoringDisable(true);
    }

    private static Command flashSuccess(LED led) {
        LEDStripPatterns patterns = new LEDStripPatterns();

        return patterns.rslSyncAnimation(led, () -> 0, () -> 255, () -> 0).ignoringDisable(true);
    }

    private static Command flashRunning(LED led) {
        LEDStripPatterns patterns = new LEDStripPatterns();

        return patterns.fullYellowCommand(led).ignoringDisable(true);
    }

    private static Command holdUntilTriggered(Trigger trigger) {
        return Commands.run(() -> {}).until(trigger);
    }

    private static Command putMessage(Supplier<String> message, boolean persist) {
        return Commands.runOnce(
                () -> UI.instance.log(message.get())); // TODO ugghhhhhhhhhhhhhhhhh fix this
    }

    private static Command selfTestCommandRunner(SelfTestCommand command, LED led) {
        return flashRunning(led)
                .alongWith(putMessage(() -> command.runningMessage, false))
                .raceWith(command.command)
                .andThen(
                        new ConditionalCommand(
                                putMessage(() -> command.successMessage, false)
                                        .alongWith(flashSuccess(led))
                                        .withTimeout(1),
                                putMessage(() -> command.failureMessage, true)
                                        .alongWith(flashFailure(led))
                                        .withTimeout(5),
                                command.success));
    }

    private static Command testSwerveModule(Swerve forRequirements, ModuleIO module, LED led) {
        double steerSettlingTimeSeconds = .5;
        double driveSettlingTimeSeconds = .5;

        return selfTestCommandRunner(
                        testNonzero(
                                " steer current",
                                module::setSteerVoltage,
                                module::getSteerCurrentAmps,
                                1,
                                12,
                                forRequirements),
                        led)
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        " steer velocity",
                                        module::setSteerVoltage,
                                        module::getSteerVelocityRadiansPerSecond,
                                        .05,
                                        2,
                                        forRequirements),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        " drive current",
                                        module::setDriveVoltage,
                                        module::getDriveCurrentAmps,
                                        1,
                                        12,
                                        forRequirements),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        " drive velocity",
                                        module::setDriveVoltage,
                                        module::getDriveVelocityRadPerSecond,
                                        .05,
                                        2,
                                        forRequirements),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testGoToSetpoint(
                                        " steer forward",
                                        module::setSteerSetpoint,
                                        module::getSteerPositionRadians,
                                        0,
                                        steerSettlingTimeSeconds,
                                        true,
                                        forRequirements),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testGoToSetpoint(
                                        " steer side",
                                        module::setSteerSetpoint,
                                        module::getSteerPositionRadians,
                                        Math.PI / 2,
                                        steerSettlingTimeSeconds,
                                        true,
                                        forRequirements),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testGoToSetpoint(
                                        " drive forward",
                                        module::setDriveSetpoint,
                                        module::getDriveVelocityRadPerSecond,
                                        30,
                                        driveSettlingTimeSeconds,
                                        false,
                                        forRequirements),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testGoToSetpoint(
                                        " drive backward",
                                        module::setDriveSetpoint,
                                        module::getDriveVelocityRadPerSecond,
                                        -30,
                                        driveSettlingTimeSeconds,
                                        false,
                                        forRequirements),
                                led))
                .finallyDo(() -> module.setDriveSetpoint(0));
    }

    private static Command swerveSelfChecks(Swerve swerve, LED led, Trigger advanceTrigger) {
        return putMessage(() -> "Waiting to begin swerve checks!", false)
                .alongWith(holdUntilTriggered(advanceTrigger))
                .andThen(putMessage(() -> "Beginning swerve checks!", false).withTimeout(2))
                .andThen(testSwerveModule(swerve, swerve.frontLeft, led))
                .andThen(testSwerveModule(swerve, swerve.frontRight, led))
                .andThen(testSwerveModule(swerve, swerve.backRight, led))
                .andThen(testSwerveModule(swerve, swerve.backLeft, led));
    }

    private static Command intakeRollerSelfChecks(
            IntakeRoller roller, LED led, Trigger advanceTrigger) {
        return putMessage(() -> "Waiting to begin intake roller checks!", false)
                .alongWith(holdUntilTriggered(advanceTrigger))
                .andThen(putMessage(() -> "Beginning intake roller checks!", false).withTimeout(2))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        "intake roller current",
                                        roller::setRollerVoltage,
                                        roller::getRollerAmps,
                                        1,
                                        2,
                                        roller),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        "intake roller velocity",
                                        roller::setRollerVoltage,
                                        roller::getAngularVelocityRPM,
                                        0.05,
                                        2,
                                        roller),
                                led))
                .andThen(roller.floorIntake().withTimeout(1))
                .andThen(roller.shooterHandoff().withTimeout(1))
                .andThen(
                        roller.idle()
                                .raceWith(
                                        putMessage(
                                                () ->
                                                        "Insert note to intake, and advance when"
                                                                + " ready!",
                                                false))
                                .alongWith(holdUntilTriggered(advanceTrigger)))
                .andThen(
                        selfTestCommandRunner(
                                new SelfTestCommand(
                                        new WaitCommand(.5),
                                        "Checking if note detected!",
                                        "Successfully detected note!",
                                        "Failed to detect note!",
                                        roller::debouncedNoteIsPresent),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                new SelfTestCommand(
                                        roller.ampShot().withTimeout(.5),
                                        "Checking if no note detected!",
                                        "Successfully detected no note!",
                                        "Failed to detect no note!",
                                        () -> !roller.debouncedNoteIsPresent()),
                                led))
                .andThen(roller.idle());
    }

    private static Command shooterSelfChecks(Shooter shooter, LED led, Trigger advanceTrigger) {
        return putMessage(() -> "Waiting to begin shooter checks!", false)
                .alongWith(holdUntilTriggered(advanceTrigger))
                .andThen(putMessage(() -> "Beginning shooter checks!", false).withTimeout(2))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        "shooter left current",
                                        shooter::setLeftVoltage,
                                        shooter::getLeftMotorAmps,
                                        1,
                                        6,
                                        shooter),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        "shooter left velocity",
                                        shooter::setLeftVoltage,
                                        shooter::getLeftVelocity,
                                        10,
                                        2,
                                        shooter),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        "shooter right current",
                                        shooter::setRightVoltage,
                                        shooter::getRightMotorAmps,
                                        1,
                                        6,
                                        shooter),
                                led))
                .andThen(
                        selfTestCommandRunner(
                                testNonzero(
                                        "shooter right velocity",
                                        shooter::setRightVoltage,
                                        shooter::getRightVelocity,
                                        10,
                                        2,
                                        shooter),
                                led))
                .andThen(shooter.subwooferShot().withTimeout(2))
                .andThen(shooter.off().withTimeout(.5));
    }

    public static Command selfTestCommand(
            BoxClimber climber,
            IntakeRoller roller,
            IntakePivot pivot,
            LED led,
            Shooter shooter,
            Swerve swerve,
            Trigger advanceTrigger) {
        return intakeRollerSelfChecks(roller, led, advanceTrigger)
                .andThen(shooterSelfChecks(shooter, led, advanceTrigger))
                .andThen(swerveSelfChecks(swerve, led, advanceTrigger));
    }
}
