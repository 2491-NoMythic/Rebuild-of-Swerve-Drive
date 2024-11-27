package frc.robot.Subsystems;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import frc.robot.Settings.Constants.ModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
public class SwerveModule {
    //The mechanical bits.
    private final TalonFX drive_Motor;
    private final TalonFX steer_Motor;
    private final CANcoder steer_Encoder;
    private final Rotation2d steer_Offset;
    private PositionDutyCycle steer_Direction = new PositionDutyCycle(0);
    private double desAngle;
    
     
  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning encoder. Also takes the wheel offset for directions.
   *
   * @param driveID 
   * @param steerID 
   * @param encID
   * @param Offler (offset, rotation2d)
   */
  public SwerveModule(String moduleName, int driveID, int steerID, int encID, Rotation2d Offler){
    drive_Motor = new TalonFX(driveID);
    steer_Motor = new TalonFX(steerID);
    steer_Encoder = new CANcoder(encID);
    steer_Offset = Offler;
  
  }
  //This section is the 'direct get' section. Everything that gets something directly from a motor is in here. 
  /**
   * Returns the current encoder angle of the steer motor.
   * @return The current encoder angle of the steer motor.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(MathUtil.inputModulus(steer_Motor.getPosition().getValue(), -0.5, 0.5));
  }
  /**
   * Returns the current encoder velocity of the drive motor.
   * @return The current velocity of the drive motor in meters/second.
   */
  public double getSpeedMetersPerSecond() {
    return (drive_Motor.getVelocity().getValue() * ModuleConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
  /**finds the curernt encoder position, it removes the current offset so we just get the raw position
   * @return
   */
    public double findOffset() {
      return MathUtil.inputModulus(
        (steer_Encoder.getPosition().getValue()+steer_Offset.getRotations()),
        -0.5,
        0.5);
    }
  //This is the get new section. It recives instructions. 
  /**
   * Returns the target angle of the wheel.
   * @return The target angle of the wheel in degrees.
   */
  public double getTargetAngle() {
    return desAngle;
  }
  //This is the state section. It controls the actual swerve drive states. 

}