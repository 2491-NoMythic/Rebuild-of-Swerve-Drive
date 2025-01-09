package frc.robot.Subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Settings.Constants.DTConstants.*;
import static frc.robot.Settings.Constants.IDConstants.*;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DrivetrainSubsystem extends SubsystemBase{
	//These are our swerve drive kinematics and Pigeon (gyroscope)
	public SwerveDriveKinematics kinList;
	private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, CANIVORE_DRIVETRAIN);

    /**
	 * These are our modules. We initialize them in the constructor.
	 * 0 = Front Left
	 * 1 = Front Right
	 * 2 = Back Left
	 * 3 = Back Right
	 */
	private final SwerveModule[] modules;
	/**
	 * These are the angles the wheels are at. This is mostly used to stop the robot without changing the angles.
	 */
	private final Rotation2d[] lastAngles;
	/**
	 * This is a number that keeps track of how many times the steering motor has rotated.
	 */
	private int accLoops;
	/**
	 * This is the odometer.
	 */
	private final SwerveDrivePoseEstimator odometer;
    public DrivetrainSubsystem(){
		 //Creating the module array and establishing lastAngles.
		modules = new SwerveModule[4];
		lastAngles = new Rotation2d[]{new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};
		//Initializing preferences 
		Preferences.initDouble("FL Offset", 0);
		Preferences.initDouble("FR Offset", 0);
		Preferences.initDouble("BL Offset", 0);
		Preferences.initDouble("BR Offset", 0);
		//The giant list of module assignments
		modules[0] = new SwerveModule("FL", FL_DRIVE_MOTOR_ID, FL_STEER_MOTOR_ID, FL_STEER_ENCODER_ID, Rotation2d.fromRotations(Preferences.getDouble("FL Offset", 0)), CANIVORE_DRIVETRAIN);
		modules[1] = new SwerveModule("FR", FR_DRIVE_MOTOR_ID, FR_STEER_MOTOR_ID, FR_STEER_ENCODER_ID, Rotation2d.fromRotations(Preferences.getDouble("FR Offset", 0)), CANIVORE_DRIVETRAIN);
		modules[2] = new SwerveModule("BL", BL_DRIVE_MOTOR_ID, BL_STEER_MOTOR_ID, BL_STEER_ENCODER_ID, Rotation2d.fromRotations(Preferences.getDouble("BL Offset", 0)), CANIVORE_DRIVETRAIN);
		modules[3] = new SwerveModule("BR", BR_DRIVE_MOTOR_ID, BR_STEER_MOTOR_ID, BR_STEER_ENCODER_ID, Rotation2d.fromRotations(Preferences.getDouble("BR Offset", 0)), CANIVORE_DRIVETRAIN);
		//Initializing the odometer
		odometer = new SwerveDrivePoseEstimator(kinList, getGyroscopeRotation(), getModulePositions(), DRIVE_ODOMETRY_ORIGIN);
    }
	//This is the main 'get' section.
	/**
	 * Gets the robot pose.
	 * @return
	 */
	public Pose2d getPose() {
		return odometer.getEstimatedPosition();
	}
	/**
	 * Returns the gyroscope rotation.
	 * @return
	 */
	public Rotation2d getGyroscopeRotation() {
		return pigeon.getRotation2d();
	}
	/**
	 * Returns a rotation 2d of the angle according to the odometer
	 * @return
	 */
	public Rotation2d getOdometryRotation() {
		return odometer.getEstimatedPosition().getRotation();
	}
	/**
	 * Returns the angle as degrees instead of rotations
	 * @return
	 */
	public double headingAsDegrees(){
		return getOdometryRotation().getDegrees();
	}
	/**
	 * Returns what directions the swerve modules are pointed in
	 * @return
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
		return positions;
	}
	/**
	 * Returns the swerve module states (a mxiture of speed and angle)
	 * @return
	 */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
		return states;
	}
	//This section has a bunch of odometry related functions.
	/**
	 * Resets the odometry of the robot.
	 * @param pose
	 */
	public void resetOdometry(Pose2d pose) {
		odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    }
	/**
	 * The function we actually use to reset the robot's 'forward'
	 * @param angleDeg
	 */
	public void zeroGyroscope(double angleDeg) {
		resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
	}
}
