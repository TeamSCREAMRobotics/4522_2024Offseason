package frc2024.utils;

import com.SCREAMLib.pid.ScreamPIDConstants;
import com.pathplanner.lib.util.PIDConstants;

public class PathplannerUtil {
    
    public static PIDConstants screamPIDConstantsToPPConstants(ScreamPIDConstants screamPIDConstants){
        return new PIDConstants(screamPIDConstants.kP(), screamPIDConstants.kI(), screamPIDConstants.kD(), screamPIDConstants.integralZone());
    }
}
