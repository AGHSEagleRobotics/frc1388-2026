// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = Units.feetToMeters(14.4); // R1 in meters per second
    public static final double DT_SECONDS = 0.02; // 20ms per tick
    public static final double DISTANCE_PER_TICK = ROBOT_MAX_SPEED * DT_SECONDS; // 20ms per tick
  }

   public static class FieldLayout {
      public static double FIELD_LENGTH = Units.inchesToMeters(651.22);
      public static double FIELD_WIDTH = Units.inchesToMeters(317.69);

      public static Translation2d CENTER_OF_HUB_BLUE = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
      public static Translation2d CENTER_OF_HUB_RED = new Translation2d(Units.inchesToMeters(534.72), Units.inchesToMeters(158.84));
    }
}
