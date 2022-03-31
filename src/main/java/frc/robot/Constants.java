// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.motors.WheeledPowerTrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class DrivetrainConstants {
		/** CAN IDs **/ 
		public static final int LEFT_MASTER_TALON_ID = 1;
		public static final int LEFT_SLAVE_TALON_ID = 2;
		public static final int RIGHT_MASTER_TALON_ID = 3;
		public static final int RIGHT_SLAVE_TALON_ID = 4;
		public static final int PIGEON_ID = 0;

		/** Power **/ 
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double GEARBOX_RATIO = 9.09090909; // 11:60, 18:30
		public static final Gearbox GEARBOX_OBJ = new Gearbox(11.0 / 60.0, 18.0 / 30.0);
		public static final double GEARBOX_OBJ_RATIO = GEARBOX_OBJ.totalReduction;
		public static final DCMotor MOTORS = DCMotor.getFalcon500(2);
		public static final double WHEEL_DIAMETER_INCHES = 6.25;
		public static final double WHEEL_DIAMETER_METERS = 0.15875;
		public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(GEARBOX_OBJ, Motor.kFalcon500, 2, WHEEL_DIAMETER_INCHES, GEARBOX_OBJ_RATIO);
		public static final double WHEELBASE_INCHES = 26.0;
		public static final double WHEELBASE_METERS = Units.inchesToMeters(WHEELBASE_INCHES);
		public static final double TRACKWIDTH_METERS = Units.inchesToMeters(25.975);
		public static final double MASS_KG = Units.lbsToKilograms(121.9);
		public static final double MOI_KGM2 = 5.120993184;

		/** System Control Values **/
		public static final double LEFT_KS = 0.66639;
		public static final double LEFT_KV = 2.5477;
		public static final double LEFT_KA = 0.2459;
		public static final SimpleMotorFeedforward LEFT_FEEDFORWARD = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV, LEFT_KA);
		public static final double LEFT_KP = 3.1833;

		public static final double RIGHT_KS = 0.63619;
		public static final double RIGHT_KV = 2.5305;
		public static final double RIGHT_KA = 0.13119;
		public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV, RIGHT_KA);
		public static final double RIGHT_KP = 2.463;

		public static final double AVG_KS = (LEFT_KS + RIGHT_KS) / 2.0;
		public static final double AVG_KV = (LEFT_KV + RIGHT_KV) / 2.0;
		public static final double AVG_KA = (LEFT_KA + RIGHT_KA) / 2.0;

		public static final SimpleMotorFeedforward AVG_FEEDFORWARD = new SimpleMotorFeedforward(AVG_KS, AVG_KV, AVG_KA);

		public static final double ANGULAR_KS = 0.49291;
		public static final double ANGULAR_KV = 0.13429;
		public static final double ANGULAR_KA = 0.0030209;
		public static final double ANGULAR_KP = 0.064151;

		/** Vision targeting **/
		public static final double AIM_KP = 0.01;
		public static final double DRIVE_KP = 0.01;

		// TEST PATH FOLLOWING TRAJECTORY
		private static final Pose2d zero_zero_StartPose = new Pose2d();
		private static final Pose2d threeMeterX_Pose = new Pose2d(3, 0, new Rotation2d());
		public static final TrajectoryConfig CALM_TRAJCONFIG = new TrajectoryConfig(2.5, 2);
		public static final TrajectoryConfig AGGRESSIVE_TRAJCONFIG = new TrajectoryConfig(4, 6);
		public static final Trajectory test3MeterForwardTraj = TrajectoryGenerator.generateTrajectory(zero_zero_StartPose, List.of(), threeMeterX_Pose, CALM_TRAJCONFIG);
	}	
}
