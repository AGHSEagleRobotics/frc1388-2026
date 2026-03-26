// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

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

    public static final double SUPPLY_CURRENT_LIMIT_BOTTOM_ROLLER = 35.0;
    public static final double SUPPLY_CURRENT_LIMIT_TOP_ROLLER = 35.0;

    public static final double bottomRollerIntakeSpeed = 1;
    public static final double bottomRollerShootingSpeed = 8;
    public static final double topRollerShootingSpeed = 10;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double DT_SECONDS = 0.02; // 20ms per tick
    public static final double DISTANCE_PER_TICK = ROBOT_MAX_SPEED * DT_SECONDS; // 20ms per tick

    public static final double ROBOT_DIMENSIONS = Units.inchesToMeters(31.25);
  }

   public static class FieldLayout {
      public static double FIELD_LENGTH = Units.inchesToMeters(651.22);
      public static double FIELD_WIDTH = Units.inchesToMeters(317.69);
   
      public static final Pose3d CENTER_OF_HUB_BLUE = new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);

      public static Pose3d CENTER_OF_HUB_RED =  new Pose3d(FIELD_LENGTH - CENTER_OF_HUB_BLUE.getX(), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);

      public static final double BLUE_ALLIANCE_ZONE = Units.inchesToMeters(156.61);
      public static final double RED_ALLIANCE_ZONE = FIELD_LENGTH - BLUE_ALLIANCE_ZONE;

      public static final Translation3d BLUE_PASSING_SPOT_LEFT = new Translation3d(
        Meters.of(Units.inchesToMeters(90)), Meters.of(FieldLayout.FIELD_WIDTH).div(2).plus(Meters.of(Units.inchesToMeters(85))), Meters.zero());
      
        public static final Translation3d BLUE_PASSING_SPOT_RIGHT = new Translation3d(
          Meters.of(Units.inchesToMeters(90)), Meters.of(FieldLayout.FIELD_WIDTH).div(2).minus(Meters.of(Units.inchesToMeters(85))), Meters.zero());

      public static final Translation3d RED_PASSING_SPOT_LEFT = new Translation3d(
        Meters.of(FIELD_LENGTH - BLUE_PASSING_SPOT_LEFT.getX()), Meters.of(FieldLayout.FIELD_WIDTH - BLUE_PASSING_SPOT_LEFT.getY()), Meters.zero());

       public static final Translation3d RED_PASSING_SPOT_RIGHT = new Translation3d(
        Meters.of(FIELD_LENGTH - BLUE_PASSING_SPOT_RIGHT.getX()), Meters.of(FieldLayout.FIELD_WIDTH - BLUE_PASSING_SPOT_RIGHT.getY()) , Meters.zero());
    }

    public static class ShooterConstants {
      public static final int SHOOT_MOTOR1_CANID = 36;
      public static final int SHOOT_MOTOR2_CANID = 37;
      public static final int KICKER_MOTOR_CANID = 42;

      public static final double SUPPLY_CURRENT_LIMIT_SHOOTER = 60.0;
      public static final double SUPPLY_CURRENT_LIMIT_KICKER = 50.0;

      public static final double TESTING_STATE_VOLTS = 4.0;

      public static final double KICKER_SHOOTING_VELOCITY = 1200.0 / 60.0;

      public static final double MANUAL_SHOOT_CLOSE = 2500.0/60.0;

      public static final double MANUAL_SHOOT_FAR = 2500.0/60.0;

      public static final double KICKER_TO_SHOOTER_RATIO = 4.0/3.0;

      public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(0, 0, 0, Rotation3d.kZero);
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_RPM = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_SHOT_RPM.put(2.08, 2100.0/60.0);
        DISTANCE_TO_SHOT_RPM.put(2.43, 2300.0/60.0);
        DISTANCE_TO_SHOT_RPM.put(2.72, 2500.0/60.0);
        DISTANCE_TO_SHOT_RPM.put(4.0, 3400.0/60.0);
      }
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_PASS_RPM = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_PASS_RPM.put(1.0, 1.0);
        DISTANCE_TO_PASS_RPM.put(2.0, 2.0);
      }
      
    }

    public static class HoodConstants {
      public static final double SUPPLY_CURRENT_LIMIT_HOOD = 20.0;

      public static final double HOOD_OFFSET = -0.975;
      public static final double HOOD_CLOSE = 0.05;
      public static final double HOOD_FAR = 0.6;
      public static final double HOOD_PASS = 0.55;
      public static final double HOOD_LIMIT_DOWN = 0.022;
      public static final double HOOD_LIMIT_UP = 0.837;
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_HOODANGLE = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_SHOT_HOODANGLE.put(1.0, 0.05); // HOOD_CLOSE — flat, close
        DISTANCE_TO_SHOT_HOODANGLE.put(2.0, 0.12);
        DISTANCE_TO_SHOT_HOODANGLE.put(3.0, 0.22);
        DISTANCE_TO_SHOT_HOODANGLE.put(4.0, 0.38);
        DISTANCE_TO_SHOT_HOODANGLE.put(5.0, 0.52);
        DISTANCE_TO_SHOT_HOODANGLE.put(6.0, 0.60); // HOOD_FAR — steep, far
      }
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_PASS_HOODANGLE = new InterpolatingDoubleTreeMap();
      static {
        DISTANCE_TO_PASS_HOODANGLE.put(1.0, 1.0);
        DISTANCE_TO_PASS_HOODANGLE.put(2.0, 2.0);
      }
    }

    public static class IntakeConstants {
      
      public static final double SUPPLY_CURRENT_LIMIT_ROLLER = 30.0;
      public static final double SUPPLY_CURRENT_LIMIT_DEPLOY = 50.0;

      public static final double INTAKE_OFFSET = -0.985;
      public static final double IDLE_STATE_ROLLER_VOLTS = 0;
      public static final double INTAKING_ROLLER_STATE_VOLTS = 12;

      public static final double POSITION_TOLERANCE = 0.185;
      public static final double DOWN_POSITION = 0.407;
      public static final double UP_POSITION = 0.04;
      public static final double HALF_WAY = 0.258;
      public static final double TESTING_VOLTS = 6;
      public static final double RAISE_INTAKE_SHOOTING_VOLTS = -1;
    }

    public static class LimelightConstants {
      public static final String SHOOTER_LIMELIGHT = "limelight-shooter";
      public static final String LEFT_LIMELIGHT = "limelight-left";
    }
  }
