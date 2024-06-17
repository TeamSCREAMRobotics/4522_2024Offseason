package com.team4522.lib.drivers;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.SimFlippable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFXSubsystem extends SubsystemBase{

    public static record CanDevice(Integer id, String canbus){
        public CanDevice(){
            this(null, null);
        }
    }

    public static record TalonFXConstants(CanDevice device, InvertedValue invert){
        public TalonFXConstants(){
            this(null, null);
        }
    }

    public static record SimState(double position, double velocity, double supplyVoltage){}

    public static class TalonFXSubystemConstants {
        public String name = "ERROR_ASSIGN_A_NAME";

        public boolean outputTelemetry = false;

        public double looperDt = 0.01;
        public double CANTimeout = 0.010; // use for important on the fly updates
        public int longCANTimeoutMs = 100; // use for constructors

        public TalonFXConstants masterConstants = new TalonFXConstants();
        public TalonFXConstants[] slaveConstants = new TalonFXConstants[0];

        public NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        public int feedbackRemoteSensorId = 99;
        public double feedbackRemoteSensorOffset = 0.0; // rotations
        public double rotorToSensorRatio = 1.0;
        public double sensorToMechRatio = 1.0;
        public double softLimitDeadband = 0.0;
        public double velocityThreshold = 0; // rps
        public double positionThreshold = 0; // rotations

        public Slot0Configs slot0 = new Slot0Configs();
        public Slot1Configs slot1 = new Slot1Configs();
        public Slot2Configs slot2 = new Slot2Configs();

        public double velocityFeedforward = 0;
        public double arbitraryFeedforward = 0;
        public double cruiseVelocity = 0; // units/s
        public double acceleration = 0; // units/s^2
        public double jerk = 0; // units/s^3
        public double rampRate = 0.0; // s
        public double maxVoltage = 12.0;

        public int supplyCurrentLimit = 60; // amps
        public boolean enableSupplyCurrentLimit = false;

        public int statorCurrentLimit = 40; // amps
        public boolean enableStatorCurrentLimit = false;

        public double maxUnitsLimit = Double.POSITIVE_INFINITY;
        public double minUnitsLimit = Double.NEGATIVE_INFINITY;
    }
    
    protected final TalonFXSubystemConstants constants;
    protected final TalonFX master;
    protected final TalonFX[] slaves;

    protected final TalonFXSimState masterSimState;

    private TalonFXConfiguration masterConfig;
    protected final TalonFXConfiguration[] slaveConfigs;

    private final StatusSignal<Double> masterPositionSignal;
    private final StatusSignal<Double> masterVelocitySignal;

    protected final double forwardSoftLimitRotations;
    protected final double reverseSoftLimitRotations;

    protected final VoltageOut voltageRequest;
    protected final PositionVoltage positionRequest;
    protected final MotionMagicVoltage motionMagicPositionRequest;
    protected final VelocityVoltage velocityRequest;
    protected final MotionMagicVelocityVoltage motionMagicVelocityRequest;

    protected double setpoint;
    protected boolean inVelocityMode = false;

    protected TalonFXSubsystem(final TalonFXSubystemConstants constants){
        this.constants = constants;
        master = new TalonFX(constants.masterConstants.device.id, constants.masterConstants.device.canbus);
        slaves = new TalonFX[constants.slaveConstants.length];
        slaveConfigs = new TalonFXConfiguration[constants.slaveConstants.length];

        masterSimState = master.getSimState();

        voltageRequest = new VoltageOut(0.0);
        positionRequest = new PositionVoltage(0.0);
        motionMagicPositionRequest = new MotionMagicVoltage(0.0);
        velocityRequest = new VelocityVoltage(0.0);
        motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0.0);

        masterPositionSignal = master.getRotorPosition();
        masterVelocitySignal = master.getRotorVelocity();

        masterConfig = new TalonFXConfiguration();

        masterConfig.Feedback.FeedbackSensorSource = constants.feedbackSensorSource;
        masterConfig.Feedback.FeedbackRemoteSensorID = constants.feedbackRemoteSensorId;

        forwardSoftLimitRotations = (constants.maxUnitsLimit - constants.softLimitDeadband);
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitRotations;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        reverseSoftLimitRotations = (constants.minUnitsLimit + constants.softLimitDeadband);
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitRotations;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterConfig.Slot0 = constants.slot0;
        masterConfig.Slot1 = constants.slot1;
        masterConfig.Slot2 = constants.slot2;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity;
        masterConfig.MotionMagic.MotionMagicAcceleration = constants.acceleration;
        masterConfig.MotionMagic.MotionMagicJerk = constants.jerk;

        masterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = constants.rampRate;
        masterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = constants.rampRate;
        masterConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = constants.rampRate;

        masterConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = constants.enableSupplyCurrentLimit;
        masterConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = constants.enableStatorCurrentLimit;

        masterConfig.MotorOutput.Inverted = constants.masterConstants.invert;
        masterConfig.Feedback.SensorToMechanismRatio = constants.sensorToMechRatio;
        masterConfig.Feedback.RotorToSensorRatio = constants.rotorToSensorRatio;
        masterConfig.MotorOutput.NeutralMode = constants.neutralMode;

        for (int i = 0; i < slaves.length; ++i) {
            slaves[i] = new TalonFX(constants.slaveConstants[i].device.id, constants.slaveConstants[i].device.canbus);

            TalonFX slave = slaves[i];
            TalonFXConfiguration slaveConfig = new TalonFXConfiguration();

            slaveConfig.MotorOutput.Inverted = constants.slaveConstants[i].invert;
            slaveConfig.MotorOutput.NeutralMode = constants.neutralMode;
            slave.setControl(new Follower(constants.masterConstants.device.id, constants.slaveConstants[i].invert != constants.masterConstants.invert));

            configSlave(slave, slaveConfig);
        }

        configMaster(masterConfig);

        stop();
    }

    public void configMaster(TalonFXConfiguration config){
        DeviceConfig.configureTalonFX(constants.name + " Master", master, config, 200);
    }

    public void configSlave(TalonFX slave, TalonFXConfiguration config){
        DeviceConfig.configureTalonFX(constants.name + " Slave", slave, config, 200);
    }

    public void setStatorCurrentLimit(double currentLimit, boolean enable) {
        changeTalonConfig((conf) -> {
            conf.CurrentLimits.StatorCurrentLimit = currentLimit;
            conf.CurrentLimits.StatorCurrentLimitEnable = enable;
            return conf;
        });
    }

    public void enableSoftLimits(boolean enable) {
        changeTalonConfig((conf) -> {
            conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
            conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
            return conf;
        });
    }

    public void setNeutralMode(NeutralModeValue mode) {
        master.setNeutralMode(mode);
        for(TalonFX slave : slaves){
            slave.setNeutralMode(mode);
        }
    }

    public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        for (int i = 0; i < slaves.length; ++i) {
            slaveConfigs[i] = configChanger.apply(slaveConfigs[i]);
        }
        masterConfig = configChanger.apply(masterConfig);
        writeConfigs();
    }

    public void writeConfigs() {
        for (int i = 0; i < slaves.length; ++i) {
            TalonFX slave = slaves[i];
            TalonFXConfiguration slaveConfig = slaveConfigs[i];
            configSlave(slave, slaveConfig);
        }
        configMaster(masterConfig);
    }

    public synchronized ControlModeValue getControlMode(){
        return master.getControlMode().asSupplier().get();
    }

    public synchronized double getRotorPosition(){
        return masterPositionSignal.asSupplier().get() * constants.rotorToSensorRatio;
    }

    public synchronized double getPosition(){
        return masterPositionSignal.asSupplier().get();
    }

    public synchronized double getRotorVelocity(){
        return masterVelocitySignal.asSupplier().get() * constants.rotorToSensorRatio;
    }

    public synchronized double getVelocity(){
        return masterVelocitySignal.asSupplier().get();
    }

    public synchronized TalonFXSimState getSimState(){
        return masterSimState;
    }

    public synchronized double getSetpoint(){
        return setpoint;
    }

    public synchronized double getError(){
        return inVelocityMode ? Math.abs(setpoint - getVelocity()) : setpoint - getPosition();
    }

    public synchronized boolean atGoal(){
        return inVelocityMode ? getError() <= constants.velocityThreshold : getError() <= constants.positionThreshold;
    }

    public synchronized void setVoltage(double volts, double voltageFeedForward){
        inVelocityMode = false;
        setMaster(voltageRequest.withOutput(volts + voltageFeedForward));
    }

    public synchronized void setVoltage(double volts){
        setVoltage(volts, 0.0);
    }

    public synchronized void setSetpointPosition(double position, double voltageFeedForward){
        setPosition(position, voltageFeedForward, false);
    }

    public synchronized void setSetpointPosition(double position){
        setSetpointPosition(position, 0.0);
    }

    public synchronized void setSetpointMotionMagicPosition(double position, double voltageFeedForward){
        setPosition(position, voltageFeedForward, true);
    }

    public synchronized void setSetpointMotionMagicPosition(double position){
        setSetpointMotionMagicPosition(position, 0.0);
    }

    public synchronized void setSetpointVelocity(double velocity, double voltageFeedForward){
        setVelocity(velocity, voltageFeedForward, false);
    }

    public synchronized void setSetpointVelocity(double velocity){
        setSetpointVelocity(velocity, 0.0);
    }

    public synchronized void setSetpointMotionMagicVelocity(double velocity, double voltageFeedForward){
        setVelocity(velocity, voltageFeedForward, true);
    }

    public synchronized void setSetpointMotionMagicVelocity(double velocity){
        setSetpointMotionMagicPosition(velocity, 0.0);
    }

    private synchronized void setPosition(double position, double voltageFeedForward, boolean motionMagic){
        setpoint = position;
        inVelocityMode = false;
        ControlRequest control = 
            motionMagic 
                ? motionMagicPositionRequest.withPosition(position).withFeedForward(voltageFeedForward)
                : positionRequest.withPosition(position).withFeedForward(voltageFeedForward);
        setMaster(control);
    }

    private synchronized void setVelocity(double velocity, double voltageFeedForward, boolean motionMagic){
        setpoint = velocity;
        inVelocityMode = true;
        ControlRequest control = 
            motionMagic 
                ? motionMagicVelocityRequest.withVelocity(velocity).withFeedForward(voltageFeedForward)
                : velocityRequest.withVelocity(velocity).withFeedForward(voltageFeedForward);
        setMaster(control);
    }

    public synchronized void setMaster(ControlRequest control){
        master.setControl(control);
    }

    public synchronized void updateSimState(SimState simState, double setpoint, boolean velocityMode){;
        masterSimState.setRawRotorPosition(simState.position);
        masterSimState.setSupplyVoltage(simState.supplyVoltage);
        masterSimState.setRotorVelocity(simState.velocity);
        this.setpoint = setpoint;
        this.inVelocityMode = velocityMode;
    }

    public synchronized void resetPosition(double position){
        master.setPosition(0.0);
    }

    public synchronized void setSupplyCurrentLimit(double value, boolean enable) {
        masterConfig.CurrentLimits.SupplyCurrentLimit = value;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        configMaster(masterConfig);
    }

    public synchronized void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
        masterConfig.CurrentLimits.SupplyCurrentLimit = value;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        master.getConfigurator().apply(masterConfig);
    }

    public synchronized void setStatorCurrentLimitUnchecked(double value, boolean enable) {
        masterConfig.CurrentLimits.StatorCurrentLimit = value;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

        master.getConfigurator().apply(masterConfig);
    }

    public synchronized void setMotionMagicConfigsUnchecked(double accel, double jerk) {
        masterConfig.MotionMagic.MotionMagicAcceleration = accel;
        masterConfig.MotionMagic.MotionMagicJerk = jerk;
        master.getConfigurator().apply(masterConfig.MotionMagic);
    }

    public synchronized void setMotionMagicConfigs(double accel, double jerk) {
        masterConfig.MotionMagic.MotionMagicAcceleration = accel;
        masterConfig.MotionMagic.MotionMagicJerk = jerk;
        
        configMaster(masterConfig);
    }

    @Override
    public void periodic() {
        if(constants.outputTelemetry){
            outputTelemetry();
        }
    }

    public void outputTelemetry(){
        SmartDashboard.putNumber(constants.name + " Position", getPosition());
        SmartDashboard.putNumberArray(constants.name + " Velocity", new double[]{getVelocity(), getVelocity() * 60.0});
        SmartDashboard.putNumber(constants.name + " Rotor Position", getRotorPosition());
        SmartDashboard.putNumberArray(constants.name + " Rotor Velocity", new double[]{getRotorVelocity(), getRotorVelocity() * 60.0});
        SmartDashboard.putNumber(constants.name + " Supply Voltage", master.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber(constants.name + " Supply Current", master.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(constants.name + " Setpoint", getSetpoint());
        SmartDashboard.putNumber(constants.name + " Error", getError());
        SmartDashboard.putBoolean(constants.name + " At Goal?", atGoal());
        SmartDashboard.putBoolean(constants.name + " In Velocity Mode", inVelocityMode);
        SmartDashboard.putString(constants.name + " Control Mode", getControlMode().toString());
    }

    public void stop() {
        master.stopMotor();
        for(TalonFX slave : slaves){
            slave.stopMotor();
        }
    }
}