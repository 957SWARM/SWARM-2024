package com.team957.comp2024.subsystems.intake;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Robot;
import com.team957.lib.math.UtilityMath;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    public abstract void setVoltage(double volts);

    public void setSetpoint(double radians) {
        setSetpointUnsafe(
                UtilityMath.clamp(
                        Constants.IntakePivotConstants.MAX_ANGLE_RADIANS,
                        Constants.IntakePivotConstants.MIN_ANGLE_RADIANS,
                        radians));
    }

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

    /**
     * Sets the feedforward control effort. On hardware, this value is sent off to the remote PID
     * controllers, and in simulation it is simply added to the model PID output.
     *
     * @param volts Signed feedforward voltage.
     */
    protected abstract void setFeedforward(double volts);

    protected abstract void setSetpointUnsafe(double setpointRadians);

    public Command goToSetpoint(Supplier<Double> setpointRadians) {
        ExponentialProfile profile =
                new ExponentialProfile(
                        Constraints.fromCharacteristics(
                                Constants.MiscConstants.saturationVoltage,
                                Constants.IntakePivotConstants.PLANT_KV,
                                Constants.IntakePivotConstants.PLANT_KA));

        ArmFeedforward feedforward =
                new ArmFeedforward(
                        Constants.IntakePivotConstants.PLANT_KS,
                        Constants.IntakePivotConstants.PLANT_KG,
                        Constants.IntakePivotConstants.PLANT_KV,
                        Constants.IntakePivotConstants.PLANT_KA);

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

                    setSetpoint(profiled.position);

                    setFeedforward(
                            feedforward.calculate(profiled.position, profiled.velocity, accel));
                });
    }
}
