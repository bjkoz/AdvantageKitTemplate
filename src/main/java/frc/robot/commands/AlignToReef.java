package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class AlignToReef extends Command {
  private Supplier<Transform2d> tagOffsetSupplier;
  private Swerve drivetrain;
  private boolean alignLeft;

  public AlignToReef(Swerve swerveSub, Supplier<Transform2d> tagOffsetSupplier, boolean alignLeft) {
    this.drivetrain = swerveSub;
    this.tagOffsetSupplier = tagOffsetSupplier;
    this.alignLeft = alignLeft;
  }

  public void initialize() {
    // transform to tag from robot pov
    /*
     * This probably isn't right and some trig is probably needed to find target position
     */
    Transform2d robotToTagOffset =
        new Transform2d(
            tagOffsetSupplier.get().getX() - Units.inchesToMeters(4.5),
            tagOffsetSupplier.get().getY(),
            tagOffsetSupplier.get().getRotation());
    // SmartDashboard.putNumber("Tag x", robotToTagOffset.getX());
    // SmartDashboard.putNumber("Tag Y", robotToTagOffset.getY());
    if (this.alignLeft) {
      robotToTagOffset =
          robotToTagOffset.plus(new Transform2d(0, -Units.inchesToMeters(7.5), Rotation2d.kZero));
    } else {
      robotToTagOffset =
          robotToTagOffset.plus(new Transform2d(0, Units.inchesToMeters(7.5), Rotation2d.kZero));
    }
    /*
     * robotToTagOffset is robot relative coords
     *
     */
    double gyroRad = this.drivetrain.getYaw().getRadians();
    Transform2d fieldRelativeTransform =
        new Transform2d(
            robotToTagOffset.getY() * Math.cos(gyroRad)
                + robotToTagOffset.getX() * Math.sin(gyroRad),
            -robotToTagOffset.getX() * Math.cos(gyroRad)
                + -robotToTagOffset.getY() * Math.sin(gyroRad),
            robotToTagOffset.getRotation());

    SmartDashboard.putNumber("Calculated X", fieldRelativeTransform.getX());
    SmartDashboard.putNumber("Calculated Y", fieldRelativeTransform.getY());

    SmartDashboard.putNumber(
        "Fancy Math X",
        robotToTagOffset.getY() * Math.cos(gyroRad) + robotToTagOffset.getX() * Math.sin(gyroRad));
    SmartDashboard.putNumber(
        "Fancy Math Y",
        robotToTagOffset.getX() * Math.cos(gyroRad) + robotToTagOffset.getY() * Math.sin(gyroRad));

    Rotation2d endRotation =
        this.drivetrain
            .getYaw()
            .minus(robotToTagOffset.getRotation())
            .plus(Rotation2d.kCW_90deg); // .plus(Rotation2d.kCW_90deg);
    SmartDashboard.putNumber("endRotation", endRotation.getDegrees());

    // CommandScheduler.getInstance().schedule(
    //   AutoBuilder.followPath(this.drivetrain.createPathplannerPath(robotToTagOffset,
    // endRotation))
    // );
    AutoBuilder.followPath(
            this.drivetrain.createPathplannerPath(
                new Transform2d(
                    robotToTagOffset.getX(),
                    -robotToTagOffset.getY(),
                    robotToTagOffset.getRotation()),
                endRotation))
        .schedule();
    SmartDashboard.putNumber("x transform", robotToTagOffset.getX());
    SmartDashboard.putNumber("y transform", robotToTagOffset.getY());
    // new DriveToPoint(drivetrain, robotToTagOffset).schedule();
  }

  public boolean isFinished() {
    return true;
  }
}
