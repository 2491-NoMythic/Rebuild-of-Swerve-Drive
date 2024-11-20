package frc.robot.Subsystems;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Rotation2d;
public class SwerveModule {
    private final TalonFX drive_Motor;
    private final TalonFX steer_Motor;
    private final CANcoder steer_Encoder;
    private final Rotation2d steer_offset;
     
  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning encoder.
   *
   * @param driveMotorChannel 
   * @param steerMotorChannel 
   * @param steerEncoderChannel
   * @param steerEncoderOffset
   */
  public SwerveModule(String moduleName, int driveID, int steerID, int encID, Rotation2d OFFLER){
    
  }
}
