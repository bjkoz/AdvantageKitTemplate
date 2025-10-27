package frc.robot;

import static frc.robot.Constants.Swerve.CTRE.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class CTREConfigs {
  public TalonFXConfiguration swerveDriveFXConfig, swerveAngleFXConfig;
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANcoderConfiguration();

    swerveCanCoderConfig.MagnetSensor.SensorDirection = kCanCoderInvert;

    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = kAngleMotorInvert;
    swerveAngleFXConfig.MotorOutput.NeutralMode = kAngleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = kAngleGearRatio;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = kAngleEnableCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = kAngleCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = kAngleCurrentThreshold;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = kAngleCurrentThresholdTime;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = angleKP;
    swerveAngleFXConfig.Slot0.kI = angleKI;
    swerveAngleFXConfig.Slot0.kD = angleKD;

    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = kDriveMotorInvert;
    swerveDriveFXConfig.MotorOutput.NeutralMode = kDriveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = kDriveGearRatio;

    /* Current Limiting */
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = kDriveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = kDriveCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = kDriveCurrentThreshold;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = kDriveCurrentThresholdTime;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = driveKP;
    swerveDriveFXConfig.Slot0.kI = driveKI;
    swerveDriveFXConfig.Slot0.kD = driveKD;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = kOpenLoopRamp;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = kOpenLoopRamp;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = kClosedLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kClosedLoopRamp;
  }
}
