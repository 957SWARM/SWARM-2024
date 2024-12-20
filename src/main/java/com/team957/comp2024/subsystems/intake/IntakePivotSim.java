package com.team957.comp2024.subsystems.intake;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.MiscConstants;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotSim extends IntakePivot {
    private final SingleJointedArmSim model =
            new SingleJointedArmSim(
                    LinearSystemId.identifyPositionSystem(
                            Constants.IntakePivotConstants.PLANT_KV,
                            Constants.IntakePivotConstants.PLANT_KA),
                    Constants.IntakePivotConstants.DRIVE_MOTOR,
                    Constants.IntakePivotConstants.GEARING_HELPER.toDoubleRatioInputToOutput(),
                    Constants.IntakePivotConstants.PIVOT_TO_TIP_METERS,
                    Constants.IntakePivotConstants.MIN_ANGLE_RADIANS,
                    Constants.IntakePivotConstants.MAX_ANGLE_RADIANS,
                    true,
                    Math.PI / 2); // assume vertical start

    private double inputVolts = 0;

    @Override
    public double getPositionRadians() {
        return model.getAngleRads();
    }

    @Override
    public double getVelocityRadiansPerSecond() {
        return model.getVelocityRadPerSec();
    }

    @Override
    public double getCurrentAmps() {
        return model.getCurrentDrawAmps();
    }

    @Override
    public double getControlEffortVolts() {
        return inputVolts;
    }

    @Override
    public void setVoltage(double volts) {
        this.inputVolts = volts;

        model.setInput(volts);
    }

    @Override
    public void periodic() {
        super.periodic();

        model.update(MiscConstants.NOMINAL_LOOP_TIME_SECONDS);
    }
}
