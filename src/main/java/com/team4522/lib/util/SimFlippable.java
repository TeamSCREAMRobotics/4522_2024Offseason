package com.team4522.lib.util;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

public class SimFlippable{

    public static double getValue(double real, double simulated){
        return Utils.isSimulation() ? simulated : real;
    }
}
