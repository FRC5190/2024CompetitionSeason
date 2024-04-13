package org.ghrobotics.frc2024;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class ShootingPosition {
  

  public double distanceToSpeaker(Pose2d robotPose, Pose2d speakerPose) {
    return Math.sqrt(Math.pow(speakerPose.getTranslation().getX() - robotPose.getTranslation().getX(), 2) + Math.pow(speakerPose.getTranslation().getY() - robotPose.getTranslation().getY(), 2));
  }

  public static final InterpolatingDoubleTreeMap SHOOTER_DISTANCE_ANGLE = new InterpolatingDoubleTreeMap();

  /**
   * 1st param is distance
   * 2nd param is armAngle
   */
  static {
    SHOOTER_DISTANCE_ANGLE.put(0.5, 10.0);  // Random Numbers for placeholder
    SHOOTER_DISTANCE_ANGLE.put(0.75, 20.0);
    SHOOTER_DISTANCE_ANGLE.put(1.0, 25.0);
    SHOOTER_DISTANCE_ANGLE.put(1.5, 34.0);
    SHOOTER_DISTANCE_ANGLE.put(2.0, 40.124);

  }



  
  public double speakerAngle() {
    double angle = ShootingPosition.SHOOTER_DISTANCE_ANGLE.get(distanceToSpeaker(null, null));

    // For Safety purposes
    // Set the correct max & min arm angles
    if (angle >= 50 || angle <= 10) {
      angle = 0.0;
    }

    return angle;
  }

  // public double armAngleToSpeaker() {
  //   double angle;

  //   angle = regressionFormula(distanceToSpeaker(null, null))

  //   return angle;
  // }

  public class Constants {
    // Red Subwoofer Pose 2d
    public static Pose2d kRedSubwooferPose = new Pose2d(16.5, 5.57, new Rotation2d(0));

    // Blue Subwoofer Pose 2d
    public static Pose2d kBlueSubwooferPose = new Pose2d(0.05, 5.57, new Rotation2d(0));
  }
}
