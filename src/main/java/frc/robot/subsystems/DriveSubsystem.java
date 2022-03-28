package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.List;

public class DriveSubsystem extends SubsystemBase {
	
	private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(LEFT_MASTER_TALON_ID);
	private final WPI_TalonFX m_leftSlave = new WPI_TalonFX(LEFT_SLAVE_TALON_ID);
	private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(RIGHT_MASTER_TALON_ID);
	private final WPI_TalonFX m_rightSlave = new WPI_TalonFX(RIGHT_SLAVE_TALON_ID);
	
	private final WPI_Pigeon2 m_imu = new WPI_Pigeon2(0);

	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
	private final DifferentialDriveKinematics m_diffDriveKinematics = new DifferentialDriveKinematics(TRACKWIDTH_METERS);
	private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_imu.getRotation2d());
	private final Field2d m_field = new Field2d();

	private final StatorCurrentLimitConfiguration CUR_LIMIT_CFG = new StatorCurrentLimitConfiguration(true, CURRENT_LIMIT, CURRENT_LIMIT + 10, 0.25);

	// Sim
	private final TalonFXSimCollection m_leftMotorSim = m_leftMaster.getSimCollection();
	private final TalonFXSimCollection m_rightMotorSim = m_rightMaster.getSimCollection();
	private final BasePigeonSimCollection m_imuSim = m_imu.getSimCollection();

	// private final LinearSystem<N2, N2, N2> m_drivePlant = LinearSystemId.identifyDrivetrainSystem(AVG_KV, AVG_KA, ANGULAR_KV, ANGULAR_KA);
	private final LinearSystem<N2, N2, N2> m_drivePlant = LinearSystemId.createDrivetrainVelocitySystem(
		MOTORS, MASS_KG, 
		WHEEL_DIAMETER_METERS / 2.0,
		TRACKWIDTH_METERS, 
		MOI_KGM2, GEARBOX_RATIO
	);
	private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
		m_drivePlant, 
		MOTORS,
		GEARBOX_RATIO,
		TRACKWIDTH_METERS,
		WHEEL_DIAMETER_METERS / 2.0,
		null
	);

	private final RamseteController m_ramsete = new RamseteController();
	private final PIDController m_leftPid = new PIDController(LEFT_KP, 0, 0);
	private final PIDController m_rightPid = new PIDController(RIGHT_KP, 0, 0);

	public DriveSubsystem() {
		// ensure slaves are brainwashed
		m_leftSlave.configFactoryDefault();
		m_rightSlave.configFactoryDefault();

		m_leftSlave.follow(m_leftMaster);
		m_leftSlave.setInverted(InvertType.FollowMaster);
		m_rightSlave.follow(m_rightMaster);
		m_rightSlave.setInverted(InvertType.FollowMaster);

		// Current Limits
		m_leftMaster.configStatorCurrentLimit(CUR_LIMIT_CFG);
		m_leftSlave.configStatorCurrentLimit(CUR_LIMIT_CFG);
		m_rightMaster.configStatorCurrentLimit(CUR_LIMIT_CFG);
		m_rightSlave.configStatorCurrentLimit(CUR_LIMIT_CFG);

		m_leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		m_rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

		m_rightMaster.setInverted(true);

		SmartDashboard.putData("Field", m_field);

		m_diffDrive.setSafetyEnabled(false);
	}

	public void teleopArcadeDrive(double xPow, double zRot) {
		m_diffDrive.arcadeDrive(xPow, zRot);
	}

	public double getLeftMeters() {
		return nativeUnitsToDistanceMeters(m_leftMaster.getSelectedSensorPosition());
	}

	public double getLeftMetersPerSec() {
		return nativeUnitsToVelocityMetersPerSec(m_leftMaster.getSelectedSensorVelocity());
	}

	public double getRightMeters() {
		return nativeUnitsToDistanceMeters(m_rightMaster.getSelectedSensorPosition());
	}

	public double getRightMetersPerSec() {
		return nativeUnitsToVelocityMetersPerSec(m_rightMaster.getSelectedSensorVelocity());
	}
	
	@Override
	public void periodic() {
		m_odometry.update(m_imu.getRotation2d(), getLeftMeters(), getRightMeters());
		m_field.setRobotPose(m_odometry.getPoseMeters());
	}

	@Override
	public void simulationPeriodic() {
		if (!RobotState.isEnabled()) return;
		
		m_leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
		m_rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

		m_driveSim.setInputs(
			m_leftMotorSim.getMotorOutputLeadVoltage(),
			-m_rightMotorSim.getMotorOutputLeadVoltage()
		);

		m_driveSim.update(0.02);

		m_leftMotorSim.setIntegratedSensorRawPosition(
			distanceToNativeUnits(m_driveSim.getLeftPositionMeters())
		);
		m_leftMotorSim.setIntegratedSensorVelocity(
			velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond())
		);

		m_rightMotorSim.setIntegratedSensorRawPosition(
			distanceToNativeUnits(-m_driveSim.getRightPositionMeters())
		);
		m_rightMotorSim.setIntegratedSensorVelocity(
			velocityToNativeUnits(-m_driveSim.getRightVelocityMetersPerSecond())
		);

		m_imuSim.setRawHeading(m_driveSim.getHeading().getDegrees());
	}

	private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(Math.PI * WHEEL_DIAMETER_METERS);
    double motorRotations = wheelRotations * GEARBOX_RATIO;
    int sensorCounts = (int)(motorRotations * 2048);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(Math.PI * WHEEL_DIAMETER_METERS);
    double motorRotationsPerSecond = wheelRotationsPerSecond * GEARBOX_RATIO;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * 2048);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / GEARBOX_RATIO;
    double positionMeters = wheelRotations * (Math.PI * WHEEL_DIAMETER_METERS);
    return positionMeters;
  }

	private double nativeUnitsToVelocityMetersPerSec(double sensorCountsPer100ms) {
		double motorRotationsPer100ms = (double) sensorCountsPer100ms / 2048;
		double motorRotationsPerSecond = motorRotationsPer100ms * 10;
		double wheelRotationsPerSecond = motorRotationsPerSecond / GEARBOX_RATIO;
		double metersPerSecond = wheelRotationsPerSecond * (Math.PI * WHEEL_DIAMETER_METERS);
		return metersPerSecond;
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void resetPose(Pose2d newPose) {
		m_leftMaster.setSelectedSensorPosition(0);
		m_rightSlave.setSelectedSensorPosition(0);

		if (RobotBase.isSimulation()) {
			m_leftMotorSim.setIntegratedSensorRawPosition(0);
			m_rightMotorSim.setIntegratedSensorRawPosition(0);

			m_driveSim.setPose(newPose);
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

		var ramseteCommand = new RamseteCommand(
			traj,
			this::getPose,
			m_ramsete,
			AVG_FEEDFORWARD,
			m_diffDriveKinematics,
			this::getWheelSpeeds,
			m_leftPid, m_rightPid,
			this::setMotorVoltages,
			this
		);

		return showOnFieldCommand.andThen(resetPoseCommand).andThen(ramseteCommand);
	}
}
