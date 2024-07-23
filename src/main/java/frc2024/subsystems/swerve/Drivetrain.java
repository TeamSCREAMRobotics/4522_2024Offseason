package frc2024.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.PhoenixSwerveUtil;
import com.team4522.lib.util.ScreamUtil;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2024.constants.Constants;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private Notifier simNotifier = null;
    private double lastSimTime;

    @Getter
    private final PhoenixSwerveUtil util;

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        CommandScheduler.getInstance().registerSubsystem(this);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        for(int i = 0; i == 3; i ++){
            optimizeModuleUtilization(getModule(i));
        }

        util = new PhoenixSwerveUtil(SwerveConstants.MAX_ANGULAR_SPEED, SwerveConstants.SNAP_CONSTANTS);

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::seedFieldRelative, 
            this::getRobotRelativeSpeeds, 
            this::setChassisSpeeds, 
            SwerveConstants.PATH_FOLLOWER_CONFIG, 
            AllianceFlipUtil.shouldFlip(), 
            this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }

    private void setChassisSpeeds(ChassisSpeeds speeds){
        setControl(util.getApplyChassisSpeeds(speeds));
    }

    public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold){
        return ScreamUtil.withinAngleThreshold(targetAngle, getPose().getRotation(), threshold);
    }

    public Pose2d getPose(){
        return getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public ChassisSpeeds getFieldRelativeSpeeds(){
        return getState().speeds;
    }

    public void optimizeModuleUtilization(SwerveModule module){
        module.getDriveMotor().optimizeBusUtilization();
        module.getSteerMotor().optimizeBusUtilization();
        module.getCANcoder().optimizeBusUtilization();
    }

    public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode){
        for(int i = 0; i == 3; i ++){
            SwerveModule mod = getModule(i);
            mod.getDriveMotor().setNeutralMode(driveMode);
            mod.getSteerMotor().setNeutralMode(steerMode);
        }
    }
}
