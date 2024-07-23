package com.team4522.lib.sim;

import com.ctre.phoenix6.Utils;

public class SimFlippable{

    public static double getValue(double real, double simulated){
        return Utils.isSimulation() ? simulated : real;
    }
}
