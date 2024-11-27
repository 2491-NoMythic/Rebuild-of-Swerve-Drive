package frc.robot.Settings;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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
    }
    public static final class ModuleConstants{
        public static final double DEGREES_TO_ROTATIONS = 2491;
        public static final double DRIVETRAIN_ROTATIONS_TO_METERS = (HardwareConstants.DRIVETRAIN_WHEEL_DIAMETER * Math.PI);
    }
    public static final class CTREConfigs {
        public TalonFXConfiguration driveMotorConfig;
        public TalonFXConfiguration steerMotorConfig;
        public CANcoderConfiguration steerEncoderConfig;
        public Pigeon2Configuration pigeon2Config;

        public CTREConfigs() {
          driveMotorConfig = new TalonFXConfiguration();
          steerMotorConfig = new TalonFXConfiguration();
          steerEncoderConfig = new CANcoderConfiguration();
        } 
    }
}
