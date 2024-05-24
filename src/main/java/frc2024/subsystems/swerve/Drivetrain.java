package frc2024.subsystems.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2024.constants.Constants;
import frc2024.constants.SwerveConstants;
import frc2024.controlboard.Controlboard;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = Constants.SIM_PERIOD_SEC;
    private Notifier simNotifier = null;
    private double lastSimTime;

    @Getter
    private final PhoenixSwerveUtil util;

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

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
            () -> false, 
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
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    private void setChassisSpeeds(ChassisSpeeds speeds){
        setControl(util.getApplyChassisSpeeds(speeds));
    }

    PIDController xController = new PIDController(10, 0, 0);
    PIDController yController = new PIDController(10, 0, 0);
    PIDController rotController = new PIDController(7, 0, 0);

    public SwerveRequest driveToPose(Pose2d pose){
        ChassisSpeeds speeds = 
            new ChassisSpeeds(
                xController.calculate(getPose().getX(), pose.getX()),
                yController.calculate(getPose().getY(), pose.getY()),
                rotController.calculate(getPose().getRotation().getRadians(), pose.getRotation().getRadians()));
        return util.getApplyChassisSpeeds(speeds);
    }

    public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold){
        return MathUtil.isNear(targetAngle.getDegrees(), getPose().getRotation().getDegrees(), threshold.getDegrees(), -180, 180);
    }

    public Pose2d getPose(){
        return getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void optimizeModuleUtilization(SwerveModule module){
        module.getDriveMotor().optimizeBusUtilization();
        module.getSteerMotor().optimizeBusUtilization();
        module.getCANcoder().optimizeBusUtilization();
    }

    @Override
    public void periodic() {}
}
