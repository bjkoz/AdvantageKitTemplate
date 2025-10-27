package frc.robot;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class LimelightWrapper {
  private String limelightName = "";

  public LimelightWrapper(String limelightName) {
    this.limelightName = limelightName;
    LimelightHelpers.setCameraPose_RobotSpace(
        limelightName,
        translationToRobot.getX(),
        translationToRobot.getY(),
        translationToRobot.getZ(),
        rotationOffset.getX(),
        rotationOffset.getY(),
        rotationOffset.getZ());
  }

  public Transform2d getNearestTagWith3DOffset() {
    double[] offsetData = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
    if (offsetData.length > 0) {

      double xOffset = offsetData[2];
      double yOffset = offsetData[0];
      double rotationOffset = offsetData[4];
      return new Transform2d(xOffset, yOffset, Rotation2d.fromDegrees(rotationOffset));
    }
    return Transform2d.kZero;
  }

  public Transform2d getTransformToBranch(boolean isLeft) {
    // sets 3d offset based on branch we're trying to align to then grabs nearest tag
    double xValue = tagToBranchOffset.getX();
    if (isLeft) {
      xValue = -xValue; // TODO might need to be changed
    }
    LimelightHelpers.SetFidcuial3DOffset(
        limelightName, xValue, tagToBranchOffset.getY(), tagToBranchOffset.getZ());
    return getNearestTagWith3DOffset();
  }
}
