package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.RobotContainer.ntInstance;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.BLModule;
import frc.robot.Constants.Swerve.BRModule;
import frc.robot.Constants.Swerve.FLModule;
import frc.robot.Constants.Swerve.FRModule;
import frc.robot.LimelightWrapper;
import frc.robot.RobotContainer;
import java.util.List;

public class Swerve extends SubsystemBase {
  PathConstraints constraints =
      new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

  private AHRS m_gryo;
  private Pigeon2 pigeon;
  private final SwerveModule[] m_swerveModules;
  private final SwerveDriveOdometry m_swerveDriveOdometry;
  private final LimelightWrapper limelight =
      new LimelightWrapper("limelight-one"); // TODO Rename this to name of limelight

  // logging SwerveModuleStates (advantagescope)
  private final StructArrayPublisher<SwerveModuleState> measuredModuleStatePub =
      ntInstance.getStructArrayTopic("Measured Module States", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> desiredModuleStatePub =
      ntInstance.getStructArrayTopic("Desired Module States", SwerveModuleState.struct).publish();
  private final StructPublisher<ChassisSpeeds> measuredChassisPub =
      ntInstance.getStructTopic("Measured Chassis Speeds", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> desiredChassisPub =
      ntInstance.getStructTopic("Desired Chassis Speeds", ChassisSpeeds.struct).publish();

  public Swerve() {
    this.m_gryo = new AHRS(NavXComType.kMXP_SPI);
    this.pigeon = new Pigeon2(0);
    // makes sure robot drives field relative
    zeroGyro();
    this.m_swerveModules =
        new SwerveModule[] {
          new SwerveModule(FLModule.kSwerveConstants),
          new SwerveModule(FRModule.kSwerveConstants),
          new SwerveModule(BLModule.kSwerveConstants),
          new SwerveModule(BRModule.kSwerveConstants)
        };
    // avoid but inverting error
    // TODO put this in its own thread
    Timer.delay(1.0);
    alignModules();
    this.m_swerveDriveOdometry =
        new SwerveDriveOdometry(
            Constants.Swerve.kSwerveKinematics, this.getYaw(), getModulePositions());
    resetPose(new Pose2d(5, 5, Rotation2d.kZero));

    configureAutoBuilder();
  }

  @Override
  public void periodic() {
    this.m_swerveDriveOdometry.update(getYaw(), getModulePositions());
    RobotContainer.m_simField.setRobotPose(this.getPose());
    SmartDashboard.putData(RobotContainer.m_simField);

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // SmartDashboard.putNumber("mod" + i,
      // m_swerveModules[i].m_driveMotor.getMotorVoltage().getValueAsDouble());
      swerveModuleStates[i] = m_swerveModules[i].getSwerveModuleState();

      // SmartDashboard.putNumber("Mod " + i,
      // m_swerveModules[i].m_angleMotor.getPosition().getValueAsDouble());
      // SmartDashboard.putNumber("Mod encoder" + i,
      // m_swerveModules[i].m_encoder.getPosition().getValueAsDouble());
    }
    // if(this.getDefaultCommand() != null)
    //     SmartDashboard.putString("Command name ", this.getDefaultCommand().getName());

    // updating publishers
    this.measuredModuleStatePub.set(swerveModuleStates);
    this.measuredChassisPub.set(
        Constants.Swerve.kSwerveKinematics.toChassisSpeeds(swerveModuleStates));
    SmartDashboard.putNumber("gyro", getYaw().getDegrees());
    Transform2d limelightTransform = limelight.getNearestTagWith3DOffset();
    SmartDashboard.putNumber("Limelight z", limelightTransform.getX());
    SmartDashboard.putNumber("Limelight strafe", limelightTransform.getY());
    SmartDashboard.putNumber("Limelight angle", limelightTransform.getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    for (int i = 0; i < this.m_swerveModules.length; i++) {
      this.m_swerveModules[i].updateSim();
    }
  }

  public void drive(Translation2d translation, double rotationRad, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    if (isFieldRelative) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotationRad, getYaw());
    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotationRad);
    }
    SwerveModuleState[] moduleStates =
        kSwerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, Constants.periodicSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, Constants.Swerve.kMaxSpeedMetersPerSec);

    for (int i = 0; i < this.m_swerveModules.length; i++) {
      this.m_swerveModules[i].setDesiredState(moduleStates[i]);
    }

    // updating publishers
    this.desiredChassisPub.set(chassisSpeeds);
    this.desiredModuleStatePub.set(moduleStates);
  }

  public Pose2d getPose() {
    return this.m_swerveDriveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d newPose) {
    this.m_swerveDriveOdometry.resetPose(newPose);
  }
  // gyro

  public void zeroGyro() {
    this.pigeon.reset();
  }

  public Rotation2d getYaw() {
    SmartDashboard.putNumber(
        "gyro offset",
        Rotation2d.fromDegrees(pigeon.getRotation2d().getDegrees())
            .plus(Rotation2d.kCCW_90deg)
            .getDegrees());
    return Rotation2d.fromDegrees(pigeon.getRotation2d().getDegrees()).plus(Rotation2d.kCCW_90deg);
  }
  // sweve modules
  public void alignModules() {
    for (SwerveModule mod : this.m_swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modPositions = new SwerveModulePosition[4];
    for (int i = 0; i < this.m_swerveModules.length; i++) {
      modPositions[i] = this.m_swerveModules[i].getModulePosition();
    }
    return modPositions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] modStates = new SwerveModuleState[4];
    for (int i = 0; i < this.m_swerveModules.length; i++) {
      modStates[i] = this.m_swerveModules[i].getSwerveModuleState();
    }
    return modStates;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(this.getModuleStates());
  }

  public void driveAuto(ChassisSpeeds PPChassisSpeeds) {
    ChassisSpeeds driveSpeeds =
        new ChassisSpeeds(
            PPChassisSpeeds.vxMetersPerSecond,
            PPChassisSpeeds.vyMetersPerSecond,
            PPChassisSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] desiredStates =
        kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(driveSpeeds, 0.02));
    for (int i = 0; i < this.m_swerveModules.length; i++) {
      this.m_swerveModules[i].setDesiredState(desiredStates[i]);
    }
  }

  public void configureAutoBuilder() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> this.driveAuto(speeds),
          new PPHolonomicDriveController(new PIDConstants(0.7, 0.5, 1.3), new PIDConstants(5)),
          config,
          () -> false,
          this);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public void stop() {
    this.drive(Translation2d.kZero, 0, false);
  }

  public PathPlannerPath createPathplannerPath(
      Transform2d relativeTransform, Rotation2d endRotation) {
    List<Waypoint> pathWaypoints =
        PathPlannerPath.waypointsFromPoses(this.getPose(), this.getPose().plus(relativeTransform));
    SmartDashboard.putNumber("Pathplanner X", this.getPose().getX());
    SmartDashboard.putNumber("Pathplanner Y", this.getPose().getY());
    SmartDashboard.putNumber("Pathplanner new X", this.getPose().plus(relativeTransform).getX());
    SmartDashboard.putNumber("Pathplanner new Y", this.getPose().plus(relativeTransform).getY());
    PathPlannerPath generatedPath =
        new PathPlannerPath(
            pathWaypoints,
            this.constraints,
            new IdealStartingState(0, this.getYaw()),
            new GoalEndState(0.0, endRotation));
    generatedPath.preventFlipping = true;
    return generatedPath;
  }

  public Transform2d tagAlignmentSupplier() {
    SmartDashboard.putNumber("limelight offset x", limelight.getNearestTagWith3DOffset().getX());
    SmartDashboard.putNumber("limelight offset z", limelight.getNearestTagWith3DOffset().getY());
    return limelight.getNearestTagWith3DOffset();
  }
}
