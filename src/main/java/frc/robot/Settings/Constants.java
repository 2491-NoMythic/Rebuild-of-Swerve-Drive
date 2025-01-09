package frc.robot.Settings;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    private Constants() {}
    public static final class HardwareConstants{
        public static final double DRIVETRAIN_WHEEL_DIAMETER = 2491;
    }
    public static final class IDConstants{
        public static final int FL_DRIVE_MOTOR_ID = 1;
        public static final int FL_STEER_MOTOR_ID = 2;
        public static final int FL_STEER_ENCODER_ID = 1;
        public static final Rotation2d FL_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final int FR_DRIVE_MOTOR_ID = 3;
        public static final int FR_STEER_MOTOR_ID = 4;
        public static final int FR_STEER_ENCODER_ID = 2;
        public static final Rotation2d FR_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final int BL_DRIVE_MOTOR_ID = 5;
        public static final int BL_STEER_MOTOR_ID = 6;
        public static final int BL_STEER_ENCODER_ID = 3;
        public static final Rotation2d BL_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final int BR_DRIVE_MOTOR_ID = 7;
        public static final int BR_STEER_MOTOR_ID = 8;
        public static final int BR_STEER_ENCODER_ID = 4;
        public static final Rotation2d BR_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final String CANIVORE_DRIVETRAIN = "Swerve";
        public static final int DRIVETRAIN_PIGEON_ID = 0;
    }
    public static final class ModuleConstants{
        public static final double DEGREES_TO_ROTATIONS = 2491;
        public static final double DRIVETRAIN_ROTATIONS_TO_METERS = (HardwareConstants.DRIVETRAIN_WHEEL_DIAMETER * Math.PI);
        public static final double MAX_VELOCITY_EMPIRCAL_RPS = 2491;
    }
    public static final class DTConstants{
        public static final Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d(5.0, 5.0, new Rotation2d());
    }
    public static final class CTREConfigs {
        private static TalonFXConfiguration getDriveMotorConfig() {
            TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
            driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
            driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 50;
            driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.8;
            return driveMotorConfig;
            }
        private static TalonFXConfiguration getSteerMotorConfig(){
            TalonFXConfiguration steerConfig = new TalonFXConfiguration();
            return steerConfig;
        }
        private static CANcoderConfiguration getSteerEncoderConfig(){
            CANcoderConfiguration enConfig = new CANcoderConfiguration();
            return enConfig;
        }
        public static final TalonFXConfiguration driveMotorConfig = getDriveMotorConfig();
        public static final TalonFXConfiguration steerMotorConfig =getSteerMotorConfig();
        public static final CANcoderConfiguration steerEncoderConfig = getSteerEncoderConfig();
    }
}