package com.team4522.lib.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.team4522.lib.pid.ScreamPIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 *  Various utility methods 
 * */
public class ScreamUtil {
    public static final double EPSILON = 1e-3;

	public static Rotation2d boundRotation(Rotation2d rotation){
		return new Rotation2d(MathUtil.angleModulus(rotation.getRadians()));
	}

	public static Rotation2d boundRotation0_360(Rotation2d rotation){
		rotation = boundRotation(rotation);
		if(rotation.getDegrees() < 0)return Rotation2d.fromDegrees(rotation.getDegrees() + 360.0);
		return rotation;
	}

	public static Rotation2d getTangent(Translation2d start, Translation2d end){
		Translation2d dist = end.minus(start);
		return new Rotation2d(dist.getX(), dist.getY());
	}

    public static boolean epsilonEquals(double a, double b, final double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

	public double getStallTorque(double stallTorque, double freeSpeed, double speed){
		return -stallTorque/freeSpeed * speed + stallTorque;
	}
	
	public static Twist2d chassisSpeedsToTwist2d(ChassisSpeeds chassisSpeeds){		
        return new Twist2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
	}

	public static boolean epsilonEquals(final Twist2d twist, final Twist2d other, double epsilon) {
        return ScreamUtil.epsilonEquals(twist.dx, other.dx, epsilon) &&
               ScreamUtil.epsilonEquals(twist.dy, other.dy, epsilon) &&
               ScreamUtil.epsilonEquals(twist.dtheta, other.dtheta, epsilon);
    }

	public static boolean epsilonEquals(final Twist2d twist, final Twist2d other) {
        return epsilonEquals(twist, other, EPSILON);
    }

    public static PPHolonomicDriveController createHolonomicDriveController(HolonomicPathFollowerConfig config){
        return new PPHolonomicDriveController(config.translationConstants, config.rotationConstants, config.period, config.maxModuleSpeed, config.driveBaseRadius);
    }

    public static boolean valueBetween(double value, double upper, double lower){
        return value < upper && value > lower;
    }

    public static boolean withinAngleThreshold(Rotation2d targetAngle, Rotation2d currentAngle, Rotation2d threshold){
        return MathUtil.isNear(targetAngle.getDegrees(), currentAngle.getDegrees(), threshold.getDegrees(), -180, 180);
    }
}
