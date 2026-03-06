// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  public static class RollerConstants {
    public static final double bottomRollerIntakeSpeed = 4;
    public static final double bottomRollerShootingSpeed = 4;
    public static final double topRollerShootingSpeed = 4;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = Units.feetToMeters(14.9); // R1 in meters per second
    public static final double DT_SECONDS = 0.02; // 20ms per tick
    public static final double DISTANCE_PER_TICK = ROBOT_MAX_SPEED * DT_SECONDS; // 20ms per tick

    public static final double ROBOT_DIMENSIONS = Units.inchesToMeters(31.25);
  }

   public static class FieldLayout {
      public static double FIELD_LENGTH = Units.inchesToMeters(651.22);
      public static double FIELD_WIDTH = Units.inchesToMeters(317.69);
   
      public static final Pose3d CENTER_OF_HUB_BLUE = new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);

      public static Pose3d CENTER_OF_HUB_RED =  new Pose3d(Units.inchesToMeters(534.72), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);

      public static final double BLUE_ALLIANCE_ZONE = Units.inchesToMeters(156.61);
      public static final double RED_ALLIANCE_ZONE = Units.inchesToMeters(FIELD_LENGTH - BLUE_ALLIANCE_ZONE);

      public static final Translation3d BLUE_PASSING_SPOT_LEFT = new Translation3d(
        Meters.of(Units.inchesToMeters(90)), Meters.of(FieldLayout.FIELD_WIDTH).div(2).plus(Meters.of(Units.inchesToMeters(85))), Meters.zero());
      
        public static final Translation3d BLUE_PASSING_SPOT_RIGHT = new Translation3d(
          Meters.of(Units.inchesToMeters(90)), Meters.of(FieldLayout.FIELD_WIDTH).div(2).minus(Meters.of(Units.inchesToMeters(85))), Meters.zero());

      public static final Translation3d RED_PASSING_SPOT_LEFT = new Translation3d(
        Meters.of(BLUE_PASSING_SPOT_LEFT.getX() - FIELD_LENGTH), Meters.of(FIELD_WIDTH).div(2).plus(Meters.of(FIELD_WIDTH - BLUE_PASSING_SPOT_LEFT.getY())), Meters.zero());

       public static final Translation3d RED_PASSING_SPOT_RIGHT = new Translation3d(
        Meters.of(BLUE_PASSING_SPOT_RIGHT.getX() - FIELD_LENGTH), Meters.of(FIELD_WIDTH).div(2).minus(Meters.of(FIELD_WIDTH - BLUE_PASSING_SPOT_RIGHT.getY())), Meters.zero());
    }

    public static class ShooterConstants {
      public static final int SHOOT_MOTOR1_CANID = 0;
      public static final int SHOOT_MOTOR2_CANID = 0;

      public static final double SHOOTING_STATE_VELOCITY = 0;
      public static final double SOTM_STATE_VELOCITY = 0;
      public static final double IDLE_STATE_VELOCITY = 0;
      public static final double PASSING_STATE_VELOCITY = 0;
      public static final double TESTING_STATE_VOLTS = 4;

      public static final double KICKER_SHOOTING_VELOCITY = 0;
      public static final double TESTING_KICKER_VOLTS = 0;

      public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(0, 0, 0, Rotation3d.kZero);
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_RPM = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_SHOT_RPM.put(1.0, 1.0);
        DISTANCE_TO_SHOT_RPM.put(2.0, 2.0);
      }
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_PASS_RPM = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_PASS_RPM.put(1.0, 1.0);
        DISTANCE_TO_PASS_RPM.put(2.0, 2.0);
      }
      
    }

    public static class HoodConstants {
      public static final double HOOD_OFFSET = 0.0;
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_HOODANGLE = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_SHOT_HOODANGLE.put(1.0, 1.0);
        DISTANCE_TO_SHOT_HOODANGLE.put(2.0, 2.0);
      }
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_PASS_HOODANGLE = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_PASS_HOODANGLE.put(1.0, 1.0);
        DISTANCE_TO_PASS_HOODANGLE.put(2.0, 2.0);
      }
    }

    public static class IntakeConstants {
      public static final double IDLE_STATE_ROLLER_VOLTS = 0;
      public static final double INTAKING_ROLLER_STATE_VOLTS = 4;

      public static final double DOWN_POSITION = 0;
      public static final double UP_POSITION = 1;
      public static final double HALF_WAY = 0.5;
      public static final double TESTING_VOLTS = 4;
    }

    public static class AutoConstants {

    public enum Objective {
      // FOURSCORERIGHT("4L4RIGHT"),
      // FOURSCORELEFT("4L4LEFT"),
      // THREESCORERIGHT("3L4Right"),
      // THREESCORELEFT("3L4Left"),
      // TWOSCORERIGHT("2L4Left"),
      // TWOSCORELEFT("2L4Right"),
      // ONESCORECENTER("1L4Cent"),
      // ONESCORELEFT("1L4Left"),
      // ONESCORERIGHT("1L4Right"),
      // LEAVE("LeaveCent"),
      // CHOREOAUTOROUTINE("choreoAutoRoutine");
      DEFAULT1("default1"),
      DEFAULT2("default2"),
      DEFAULT3("default3");


      public static final Objective Default = DEFAULT1;

      private String m_dashboardDescript; // This is what will show on dashboard

      private Objective(String dashboardDescript) {
        m_dashboardDescript = dashboardDescript;
      }

      public String getDashboardDescript() {
        return m_dashboardDescript;
      }
    }

  }
}
