package com.team957.comp2024.subsystems.intake;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.IntakePivotConstants;
import com.team957.comp2024.Robot;
import com.team957.comp2024.UI;
import com.team957.lib.math.UtilityMath;
import edu.wpi.first.math.controller.ArmFeedforward;
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

    public void setVoltage(double volts) {
        // spotless:off
        setVoltageUnsafe(
                (volts < 0 && (getPositionRadians() < IntakePivotConstants.MIN_ANGLE_RADIANS))
            || 
                (volts > 0 && (getPositionRadians() > IntakePivotConstants.MAX_ANGLE_RADIANS))
            ? 0 : volts);
        // spotless:on
    }

    protected abstract void setVoltageUnsafe(double volts);

    @Log.NT
    public abstract double getPositionRadians();

    @Log.NT
    public abstract double getVelocityRadiansPerSecond();

    @Log.NT
    public abstract double getCurrentAmps();

    @Log.NT
    public abstract double getControlEffortVolts();

    protected boolean pivotOnboardClosedLoop = false;

    @Log.NT
    public boolean isOnboardClosedLoop() {
        return pivotOnboardClosedLoop;
    }

    @Override
    public void periodic() {
        UI.instance.setIntakeAngle(getPositionRadians());
    }

    /**
     * Sets the feedforward control effort. On hardware, this value is sent off to the remote PID
     * controllers, and in simulation it is simply added to the model PID output.
     *
     * @param volts Signed feedforward voltage.
     */
    public void setFeedforwardAndSetpoint(double volts, double setpointRadians) {
        setFeedforwardAndSetpointUnsafe(
                volts,
                UtilityMath.clamp(
                        IntakePivotConstants.MAX_ANGLE_RADIANS,
                        IntakePivotConstants.MIN_ANGLE_RADIANS,
                        setpointRadians));
    }

    protected abstract void setFeedforwardAndSetpointUnsafe(double volts, double setpointRadians);

    public Command getSysIdQuasistatic(boolean forward) {
        return routine.quasistatic(forward ? Direction.kForward : Direction.kReverse);
    }

    public Command getSysIdDynamic(boolean forward) {
        return routine.dynamic(forward ? Direction.kForward : Direction.kReverse);
    }

    public Command goToSetpoint(Supplier<Double> setpointRadians) {
        ExponentialProfile profile =
                new ExponentialProfile(
                        Constraints.fromCharacteristics(
                                Constants.MiscConstants.saturationVoltage,
                                IntakePivotConstants.PLANT_KV,
                                IntakePivotConstants.PLANT_KA));

        ArmFeedforward feedforward =
                new ArmFeedforward(
                        IntakePivotConstants.PLANT_KS,
                        IntakePivotConstants.PLANT_KG,
                        IntakePivotConstants.PLANT_KV,
                        IntakePivotConstants.PLANT_KA);

        // mutating existing objects to avoid allocations
        State current = new State();
        State goal = new State();

        return run(
                () -> {
                    current.position = getPositionRadians();
                    current.velocity = getVelocityRadiansPerSecond();

                    goal.position = setpointRadians.get();
                    goal.velocity = 0;

                    var profiled = profile.calculate(Robot.kDefaultPeriod, current, goal);

                    double accel = (profiled.velocity - current.velocity) / Robot.kDefaultPeriod;

                    log("profiled", profiled.position);
                    log("profiledV", profiled.velocity);

                    setFeedforwardAndSetpoint(
                            feedforward.calculate(profiled.position, profiled.velocity, accel),
                            profiled.position);
                });
    }

    public static IntakePivot getIntakePivot(boolean isReal) {
        if (!isReal) return new IntakePivotSim();
        else return new IntakePivotHW();
    }
}
