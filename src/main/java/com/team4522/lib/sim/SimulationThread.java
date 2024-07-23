package com.team4522.lib.sim;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.drivers.TalonFXSubsystem.SimState;
import com.team4522.lib.pid.ScreamPIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc2024.constants.Constants;

public class SimulationThread {

    private SimInterface simInterface;
    private Notifier simNotifier = null;
    private double lastSimTime;

    private DoubleSupplier voltageSupplier;

    private Consumer<SimState> stateConsumer;

    private double periodSec;
    
    public SimulationThread(SimInterface simInterface, DoubleSupplier voltageSupplier, Consumer<SimState> stateConsumer, double periodSec){
        this.simInterface = simInterface;
        this.stateConsumer = stateConsumer;
        this.periodSec = periodSec;
        startSimThread();
    }

    public void startSimThread(){
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            simInterface.update(deltaTime);
            simInterface.setInputVoltage(voltageSupplier.getAsDouble());
            stateConsumer.accept(
                new SimState(
                    simInterface.getPosition(), 
                    simInterface.getVelocity(), 
                    RobotController.getBatteryVoltage()));
        });
        simNotifier.startPeriodic(periodSec);
    }
}
