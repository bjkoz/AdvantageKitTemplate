package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
// import static frc.robot.Constants.Swerve;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  public TalonFX m_angleMotor, m_driveMotor;
  public CANcoder m_encoder;
  private DutyCycleOut m_driveDutyCycle = new DutyCycleOut(0);
  private PositionVoltage m_anglePositionVoltage = new PositionVoltage(0);

  // simulation
  private TalonFXSimState m_angleMotorSim, m_driveMotorSim;
  private CANcoderSimState m_encoderSim;

  private double angleOffset = 0;

  public SwerveModule(SwerveConstants moduleIDConstants) {
    m_driveMotor = new TalonFX(moduleIDConstants.kDriveMotorID());
    m_angleMotor = new TalonFX(moduleIDConstants.kAngleMotorID());
    m_encoder = new CANcoder(moduleIDConstants.kCancoderID());

    m_angleMotor
        .getConfigurator()
        .apply(frc.robot.Constants.Swerve.kCTREConfigs.swerveAngleFXConfig);
    m_driveMotor
        .getConfigurator()
        .apply(frc.robot.Constants.Swerve.kCTREConfigs.swerveDriveFXConfig);
    m_encoder.getConfigurator().apply(frc.robot.Constants.Swerve.kCTREConfigs.swerveCanCoderConfig);
    // zero drive encoder
    m_driveMotor.getConfigurator().setPosition(0);
    angleOffset = moduleIDConstants.kAngleOffset();
  }
  // sets integrated encoder to absolute position
  // eliminates need to manually align swerves
  public void resetToAbsolute() {
    // this.m_angleMotor.setPosition(0);
    // difference between absolute angle and relative angle
    double absolutePosition =
        m_encoder.getPosition().getValueAsDouble()
            - m_encoder.getAbsolutePosition().getValueAsDouble();
    // double absoultePosition = m_encoder.getPosition().getValueAsDouble() -
    // m_angleMotor.getPosition().getValueAsDouble();

    this.m_angleMotor.setPosition(m_encoder.getPosition().getValueAsDouble() - angleOffset);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(getAngle());
    setAngle(desiredState.angle);
    // takes difference between angle setpoint and current angle
    // then take cos of angle to limit speed when angle difference is large
    desiredState.cosineScale(getAngle());
    setDriveSpeed(desiredState.speedMetersPerSecond);
  }

  public void setAngle(Rotation2d angle) {
    m_angleMotor.setControl(m_anglePositionVoltage.withPosition(angle.getRotations()));
  }

  public void setDriveSpeed(double speedMetersPerSecond) {
    m_driveDutyCycle.Output = speedMetersPerSecond / Swerve.kMaxSpeedMetersPerSec;
    m_driveMotor.setControl(m_driveDutyCycle);
  }
  // gets module positions for odometry
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveDistMeters(), getAngle());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble());
  }

  public double getDriveDistMeters() {
    return m_driveMotor.getPosition().getValueAsDouble() * Swerve.kWheelCircumference;
  }

  public SwerveModuleState getSwerveModuleState() {
    double speedMetersPerSecond =
        this.m_driveMotor.getVelocity().getValueAsDouble() * Constants.Swerve.kWheelCircumference;
    Rotation2d moduleAngle = this.getAngle();
    return new SwerveModuleState(speedMetersPerSecond, moduleAngle);
  }

  public void updateSim() {}
}
