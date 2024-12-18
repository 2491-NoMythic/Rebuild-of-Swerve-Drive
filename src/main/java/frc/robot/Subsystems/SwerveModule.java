package frc.robot.Subsystems;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import frc.robot.Settings.Constants.ModuleConstants;
import frc.robot.Settings.Constants.CTREConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
public class SwerveModule {
    //The mechanical bits.
    private final TalonFX drive_Motor;
    private final TalonFX steer_Motor;
    private final CANcoder steer_Encoder;
    private final Rotation2d steer_Offset;
    private TalonFXConfiguration drive_Configuration;
    private TalonFXConfiguration steer_Configuration;
    private CANcoderConfiguration enc_Configuration;
    //variables
    private double desAngle;
    private double desSpeed;
    //controls (dm: drive motor, sm: steer motor)
    private VelocityVoltage dmControl = new VelocityVoltage(0);
    private PositionDutyCycle smControl = new PositionDutyCycle(0);
    private NeutralOut stopped = new NeutralOut();
  /**
   * Constructs a SwerveModule with a drive motor, steering motor, and steering encoder.
   * Also takes a rotation 2d offset for directions and a canivore name. 
   *
   * @param driveID (drive motor id, integer)
   * @param steerID (steer motor id, integer)
   * @param encID (steer encoder id, integer)
   * @param Offler (offset, rotation2d. Also the Discworld's god of crocodiles.)
   * @parama canivoreName (name of the canivore)
   */
  public SwerveModule(String moduleName, int driveID, int steerID, int encID, Rotation2d Offler, String canivoreName){
    //TalonFXConfiguration dmConfig, 
     //TalonFXConfiguration smConfig, CANcoderConfiguration seConfig,
    drive_Motor = new TalonFX(driveID, canivoreName);
    steer_Motor = new TalonFX(steerID, canivoreName);
    steer_Encoder = new CANcoder(encID, canivoreName);
    drive_Configuration = CTREConfigs.driveMotorConfig;
    steer_Configuration = CTREConfigs.steerMotorConfig;
    enc_Configuration = CTREConfigs.steerEncoderConfig;
    steer_Offset = Offler;

    enc_Configuration.MagnetSensor.MagnetOffset = -steer_Offset.getRotations();
    steer_Configuration.Feedback.FeedbackRemoteSensorID = encID;
    drive_Motor.getConfigurator().apply(drive_Configuration);
    steer_Motor.getConfigurator().apply(steer_Configuration);
    steer_Encoder.getConfigurator().apply(enc_Configuration);
  }
  //This section is the 'direct get' section. Everything that gets something directly from a motor is in here.
  public TalonFX getDriveMotor(){
    return drive_Motor;
  }

  public TalonFX getSteerMotor(){
    return steer_Motor;
  }
  /**
   * Returns the current encoder distance of the drive motor.
   * @return The current distance of the drive motor in meters.
   */
  public double getDriveDistanceMeters() {
    return (drive_Motor.getPosition().getValue() * ModuleConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
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
   * @return The current encoder position.
   */
    public double findOffset() {
      return MathUtil.inputModulus(
        (steer_Encoder.getPosition().getValue()+steer_Offset.getRotations()),
        -0.5,
        0.5);
    }
    /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getSpeedMetersPerSecond(), getRotation());
  }
  /**
   * Returns the current position of the module. Includes the modules rotation and the modules distance driven.
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistanceMeters(), getRotation());
  }
  //This is the direction section. It recives instructions and sets the desired state of the swerve module. 
  /**
   * Returns the target angle of the wheel.
   * @return The target angle of the wheel in degrees.
   */
  public double getTargetAngle() {
    return desAngle;
  }
  /**
  * Returns the target speed of the wheel.
  * @return The target speed of the wheel in meters/second.
  */
  public double getTargetSpeedMetersPerSecond() {
    return desSpeed;
  }
  public void setDesiredState(SwerveModuleState desState){
    if (desState.angle == null){
      DriverStation.reportWarning("Error: module angle should be a number, not null.", true);
    }
    //Optimization is to stop the wheel from spinning more than 90 degrees. 
    SwerveModuleState state = SwerveModuleState.optimize(desState, getRotation());
    desAngle = MathUtil.inputModulus(state.angle.getRotations(), -0.5, -0.5);
    if( Math.abs(desSpeed) <= 0.001){
      drive_Motor.setControl(stopped);
    }
    else{
      drive_Motor.setControl(dmControl.withVelocity(desSpeed).withFeedForward((desSpeed/ModuleConstants.MAX_VELOCITY_EMPIRCAL_RPS)*12));
    }
    steer_Motor.setControl(smControl.withPosition(desAngle));
  }
}