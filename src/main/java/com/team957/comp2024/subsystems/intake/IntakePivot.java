package com.team957.comp2024.subsystems.intake;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.IntakePivotConstants;
import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.UI;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

/**
 * Subsystem for the pivot of the intake alone.
 *
 * <p>Splitting the roller/pivot this way seems odd at first, but it's because they are actually
 * completely independent of each other. Putting them in the same subsystem would complicate writing
 * concise Commands for them.
 */
public abstract class IntakePivot implements Subsystem, Logged {
    private final SysIdRoutine routine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) -> {
                                setVoltage(volts.magnitude());
                            },
                            null,
                            this));

    public IntakePivot() {
        register();
    }

    public abstract void setVoltage(double volts);

    @Log.NT
    public abstract double getPositionRadians();

    @Log.NT
    public abstract double getUnoffsetPositionRadians();

    @Log.NT
    public abstract double getVelocityRadiansPerSecond();

    @Log.NT
    public abstract double getCurrentAmps();

    @Log.NT
    public abstract double getControlEffortVolts();

    @Override
    public void periodic() {
        UI.instance.setIntakeAngle(getPositionRadians());

        Command activeCommand = getCurrentCommand();

        if (activeCommand != null) log("activeCommand", activeCommand.getName());
    }

    public Command getSysIdQuasistatic(boolean forward) {
        return routine.quasistatic(forward ? Direction.kForward : Direction.kReverse);
    }

    public Command getSysIdDynamic(boolean forward) {
        return routine.dynamic(forward ? Direction.kForward : Direction.kReverse);
    }

    public Command goToSetpoint(Supplier<Double> setpointRadians) {
        PID pid = new PID(IntakePivotConstants.PID_CONSTANTS, 0, false);

        return run(
                () -> {
                    pid.setSetpoint(MathUtil.angleModulus(setpointRadians.get()));
                    setVoltage(pid.calculate(MathUtil.angleModulus(getPositionRadians())));
                });
    }

    public Command profiledGoToSetpoint(Supplier<Double> setpointRadians) {
        ExponentialProfile profile =
                new ExponentialProfile(
                        Constraints.fromCharacteristics(
                                IntakePivotConstants.INTAKE_PIVOT_PROFILE_CONTROL_EFFORT,
                                IntakePivotConstants.PLANT_KV,
                                IntakePivotConstants.PLANT_KA));

        // ArmFeedforward feedforward =
        //         new ArmFeedforward(
        //                 IntakePivotConstants.PLANT_KS,
        //                 IntakePivotConstants.PLANT_KG,
        //                 IntakePivotConstants.PLANT_KV,
        //                 IntakePivotConstants.PLANT_KA);

        // mutating existing objects to avoid allocations
        State current = new State();
        State goal = new State();

        return goToSetpoint(
                        () -> {
                            current.position = getPositionRadians();
                            current.velocity = getVelocityRadiansPerSecond();

                            goal.position = setpointRadians.get();
                            goal.velocity = 0;

                            var profiled =
                                    profile.calculate(
                                            MiscConstants.NOMINAL_LOOP_TIME_SECONDS, current, goal);

                            // double accel =
                            //         (profiled.velocity - current.velocity)
                            //                 / MiscConstants.NOMINAL_LOOP_TIME_SECONDS;

                            // log("profiled", profiled.position);
                            // log("profiledV", profiled.velocity);

                            return profiled.position;
                        })
                .withName("goToSetpoint");
    }

    public Command toFloor() {
        return profiledGoToSetpoint(() -> Constants.IntakePivotConstants.FLOOR_INTAKE_ANGLE_RADIANS)
                .withName("toFloor");
    }

    public Command toStow() {
        return profiledGoToSetpoint(() -> Constants.IntakePivotConstants.STOW_INTAKE_ANGLE_RADIANS)
                .withName("toStow");
    }

    public Command toHandoff() {
        return profiledGoToSetpoint(
                        () -> Constants.IntakePivotConstants.HANDOFF_INTAKE_ANGLE_RADIANS)
                .withName("toHandoff");
    }

    public Command toAmp() {
        return profiledGoToSetpoint(() -> Constants.IntakePivotConstants.AMP_INTAKE_ANGLE_RADIANS)
                .withName("toAmp");
    }

    public static IntakePivot getIntakePivot(boolean isReal) {
        if (!isReal) return new IntakePivotSim();
        else return new IntakePivotHW(); // not implemented
    }
}
