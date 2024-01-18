package com.team957.comp2024.util;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkMax;
import java.util.ArrayList;

public class SparkMaxUtils {
    // object allocation bad!
    private static final ArrayList<String> faultsBuffer = new ArrayList<>();

    public static String[] getFaultsAsStringArray(CANSparkMax sMax, boolean sticky) {
        faultsBuffer.clear();

        for (FaultID fault : FaultID.values()) {
            if ((sticky) ? sMax.getStickyFault(fault) : sMax.getFault(fault))
                faultsBuffer.add(fault.toString());
        }

        return faultsBuffer.toArray(new String[] {});
    }
}
