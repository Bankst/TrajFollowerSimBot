package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.OscarRamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.SmartMCSim;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.motors.WheeledPowerTrain;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.List;

public class DriveSubsystem extends SubsystemBase {

	private final CANTalonFX m_leftMaster = new CANTalonFX(LEFT_MASTER_TALON_ID);
	private final CANTalonFX m_leftSlave = new CANTalonFX(LEFT_SLAVE_TALON_ID);
	private final CANTalonFX m_rightMaster = new CANTalonFX(RIGHT_MASTER_TALON_ID);
	private final CANTalonFX m_rightSlave = new CANTalonFX(RIGHT_SLAVE_TALON_ID);

	private final WPI_Pigeon2 m_imu = new WPI_Pigeon2(0);

	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
	private final DifferentialDriveKinematics m_diffDriveKinematics = new DifferentialDriveKinematics(TRACKWIDTH_METERS);
	private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_imu.getRotation2d());
	private final Field2d m_field = new Field2d();

	// Sim
	// private final TalonFXSimCollection m_leftMotorSim = m_leftMaster.getBaseController().getSimCollection();
	// private final TalonFXSimCollection m_rightMotorSim = m_rightMaster.getBaseController().getSimCollection();
	private final SmartMCSim m_leftMotorSim = m_leftMaster.getSim();
	private final SmartMCSim m_rightMotorSim = m_rightMaster.getSim();
	private final BasePigeonSimCollection m_imuSim = m_imu.getSimCollection();

	private final WheeledPowerTrain m_powertrain = POWER_TRAIN;

	// private final LinearSystem<N2, N2, N2> m_drivePlant =
	// LinearSystemId.identifyDrivetrainSystem(AVG_KV, AVG_KA, ANGULAR_KV,
	// ANGULAR_KA);
	private final LinearSystem<N2, N2, N2> m_drivePlant = LinearSystemId.createDrivetrainVelocitySystem(
			MOTORS, MASS_KG,
			WHEEL_DIAMETER_METERS / 2.0,
			TRACKWIDTH_METERS,
			MOI_KGM2, GEARBOX_RATIO);
	private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
			m_drivePlant,
			MOTORS,
			GEARBOX_RATIO,
			TRACKWIDTH_METERS,
			WHEEL_DIAMETER_METERS / 2.0,
			null);

	private final RamseteController m_ramsete = new RamseteController();
	private final PIDController m_leftPid = new PIDController(LEFT_KP, 0, 0);
	private final PIDController m_rightPid = new PIDController(RIGHT_KP, 0, 0);

	public DriveSubsystem() {
		// ensure slaves are brainwashed
		m_leftSlave.wipeSettings();
		m_rightSlave.wipeSettings();

		m_leftSlave.follow(m_leftMaster.getBaseController());
		m_leftSlave.getBaseController().setInverted(InvertType.FollowMaster);
		m_rightSlave.follow(m_rightMaster.getBaseController());
		m_rightSlave.getBaseController().setInverted(InvertType.FollowMaster);

		// Current Limits
		m_leftMaster.limitOutputCurrent(CURRENT_LIMIT);
		m_leftSlave.limitOutputCurrent(CURRENT_LIMIT);
		m_rightMaster.limitOutputCurrent(CURRENT_LIMIT);
		m_rightSlave.limitOutputCurrent(CURRENT_LIMIT);

		m_rightMaster.setInverted(true);

		SmartDashboard.putData("Field", m_field);

		m_diffDrive.setSafetyEnabled(false);
	}

	public void teleopArcadeDrive(double xPow, double zRot) {
		m_diffDrive.arcadeDrive(xPow, zRot);
	}

	public double getLeftMeters() {
		var rots = m_leftMaster.getSensorPosition();
		SmartDashboard.putNumber("DTSubsys/LeftMotorRots", rots);
		var meters = POWER_TRAIN.calcWheelDistanceMeters(rots);
		return meters;
	}

	public double getLeftMetersPerSec() {
		var veloRpm = m_leftMaster.getSensorVelocity();
		SmartDashboard.putNumber("DTSubsys/LeftMotorVeloRpm", veloRpm);
		var veloMps = m_powertrain.calcMetersPerSec(veloRpm);
		SmartDashboard.putNumber("DTSubsys/LeftVeloMps", veloMps);
		return veloMps;
	}

	public double getRightMeters() {
		var rots = m_rightMaster.getSensorPosition();
		SmartDashboard.putNumber("DTSubsys/LeftMotorRots", rots);
		var meters = m_powertrain.calcWheelDistanceMeters(rots);
		return meters;
	}

	public double getRightMetersPerSec() {
		var veloRpm = m_rightMaster.getSensorVelocity();
		SmartDashboard.putNumber("DTSubsys/RightMotorVeloRpm", veloRpm);
		var veloMps = m_powertrain.calcMetersPerSec(veloRpm);
		SmartDashboard.putNumber("DTSubsys/RightVeloMps", veloMps);
		return veloMps;
	}

	@Override
	public void periodic() {
		getLeftMetersPerSec();
		getRightMetersPerSec();

		m_odometry.update(m_imu.getRotation2d(), getLeftMeters(), getRightMeters());
		m_field.setRobotPose(m_odometry.getPoseMeters());
	}

	@Override
	public void simulationPeriodic() {
		if (!RobotState.isEnabled())
			return;

		m_leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
		m_rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

		m_driveSim.setInputs(
				m_leftMotorSim.getOutputVoltage(),
				-m_rightMotorSim.getOutputVoltage());

		m_driveSim.update(0.02);

		var leftMeters = m_driveSim.getLeftPositionMeters();
		var leftMps = m_driveSim.getLeftVelocityMetersPerSecond();
		SmartDashboard.putNumber("Sim/LeftMeters", leftMeters);
		SmartDashboard.putNumber("Sim/LeftMps", leftMps);

		var leftSensorPosition = m_powertrain.calcEncoderRotationsFromMeters(leftMeters);
		var leftSensorVelocity = m_powertrain.calcEncoderRpmFromMetersPerSec(leftMps);
		m_leftMotorSim.setSensorPosition(leftSensorPosition);
		m_leftMotorSim.setSensorVelocity(leftSensorVelocity);

		var rightMeters = -m_driveSim.getRightPositionMeters();
		var rightMps = -m_driveSim.getRightVelocityMetersPerSecond();
		SmartDashboard.putNumber("Sim/RightMeters", rightMeters);
		SmartDashboard.putNumber("Sim/RightMps", rightMps);

		var rightSensorPosition = m_powertrain.calcEncoderRotationsFromMeters(rightMeters);
		var rightSensorVelocity = m_powertrain.calcEncoderRpmFromMetersPerSec(rightMps);
		m_rightMotorSim.setSensorPosition(rightSensorPosition);
		m_rightMotorSim.setSensorVelocity(rightSensorVelocity);

		m_imuSim.setRawHeading(m_driveSim.getHeading().getDegrees());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void resetPose(Pose2d newPose) {
		m_leftMaster.rezeroSensor();
		m_rightSlave.rezeroSensor();

		if (RobotBase.isSimulation()) {
			m_leftMotorSim.setSensorPosition(0);
			m_rightMotorSim.setSensorPosition(0);

			m_driveSim.setPose(newPose);
			m_driveSim.update(0.02);
		}

		m_odometry.resetPosition(newPose, m_imu.getRotation2d());
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftMetersPerSec(), getRightMetersPerSec());
	}

	public void setMotorVoltages(double leftVolts, double rightVolts) {
		m_leftMaster.setVoltage(leftVolts);
		m_rightMaster.setVoltage(rightVolts);
	}



	public CommandBase getRamseteCommand(Trajectory traj, boolean showPath) {
		var showOnFieldCommand = new InstantCommand(() -> {
			if (showPath) {
				m_field.getObject("RamseteCommandTraj").setTrajectory(traj);
			} else {
				m_field.getObject("RamseteCommandTraj").setPoses(List.of());
			}
		}, this);

		var resetPoseCommand = new InstantCommand(() -> {
			resetPose(traj.getInitialPose());
		}, this).withTimeout(0.125);

		var noPid = new PIDController(0, 0, 0);

		var ramseteCommand = new OscarRamseteCommand(
			traj, this::getPose, m_ramsete, 
			LEFT_FEEDFORWARD, RIGHT_FEEDFORWARD, 
			m_diffDriveKinematics, 
			this::getWheelSpeeds, 
			// noPid, noPid,
			m_leftPid, m_rightPid,
			this::setMotorVoltages,
			this
		);

		return showOnFieldCommand.andThen(resetPoseCommand).andThen(ramseteCommand);
	}
}
