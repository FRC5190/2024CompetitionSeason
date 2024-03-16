package org.ghrobotics.frc2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShootingPosition {
  

  public double distanceToSpeaker(Pose2d robotPose, Pose2d speakerPose) {
    return Math.sqrt(Math.pow(speakerPose.getTranslation().getX() - robotPose.getTranslation().getX(), 2) + Math.pow(speakerPose.getTranslation().getY() - robotPose.getTranslation().getY(), 2));
  }

  /**
   * Returns expected angle based on distance to speaker
   * @param x Distance to the speaker
   * @return Arm Angle to speaker
   */
  public double regressionFormula(double x) {
    double y = (2 * x) + 1;

    return y;
  }

  // public double armAngleToSpeaker() {
  //   double angle;

  //   angle = regressionFormula(distanceToSpeaker(null, null))

  //   return angle;
  // }

  public class Constatnts {
    // Red Subwoofer Pose 2d
    public static Pose2d redSubwooferPose = new Pose2d(16.5, 5.57, new Rotation2d(0));

    // Blue Subwoofer Pose 2d
    public static Pose2d blueSubwooferPose = new Pose2d(0.05, 5.57, new Rotation2d(0));
  }
}
