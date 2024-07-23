package com.team4522.lib.sim;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimWrappers {

    public static class DCMotorSimWrapper implements SimInterface {
        private final DCMotorSim sim;

        public DCMotorSimWrapper(DCMotorSim sim) {
            this.sim = sim;
        }

        @Override
        public void update(double deltaTime) {
            sim.update(deltaTime);
        }

        @Override
        public void setInputVoltage(double voltage) {
            sim.setInputVoltage(voltage);
        }

        @Override
        public double getPosition() {
            return sim.getAngularPositionRotations();
        }

        @Override
        public double getVelocity() {
            return sim.getAngularVelocityRPM();
        }
    }

    public static class ElevatorSimWrapper implements SimInterface {
        private final ElevatorSim sim;

        public ElevatorSimWrapper(ElevatorSim sim) {
            this.sim = sim;
        }

        @Override
        public void update(double deltaTime) {
            sim.update(deltaTime);
        }

        @Override
        public void setInputVoltage(double voltage) {
            sim.setInputVoltage(voltage);
        }

        @Override
        public double getPosition() {
            return sim.getPositionMeters();
        }

        @Override
        public double getVelocity() {
            return sim.getVelocityMetersPerSecond();
        }
    }

    public static class SingleJointedArmSimWrapper implements SimInterface {
        private final SingleJointedArmSim sim;

        public SingleJointedArmSimWrapper(SingleJointedArmSim sim) {
            this.sim = sim;
        }

        @Override
        public void update(double deltaTime) {
            sim.update(deltaTime);
        }

        @Override
        public void setInputVoltage(double voltage) {
            sim.setInputVoltage(voltage);
        }

        @Override
        public double getPosition() {
            return sim.getAngleRads();
        }

        @Override
        public double getVelocity() {
            return sim.getVelocityRadPerSec();
        }
    }

    public static class FlywheelSimWrapper implements SimInterface {
        private final FlywheelSim sim;

        public FlywheelSimWrapper(FlywheelSim sim) {
            this.sim = sim;
        }

        @Override
        public void update(double deltaTime) {
            sim.update(deltaTime);
        }

        @Override
        public void setInputVoltage(double voltage) {
            sim.setInputVoltage(voltage);
        }

        @Override
        public double getPosition() {
            return 0.0;
        }

        @Override
        public double getVelocity() {
            return sim.getAngularVelocityRPM();
        }
    }
}
