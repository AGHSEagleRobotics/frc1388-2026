package frc.robot.shotlib;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.ShooterConstants;
import frc.robot.shotlib.ShootOnTheFlyCalculator.InterceptSolution;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class ShotCalculator extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    private Pose3d targetLocation = Pose3d.kZero;

    private double targetDistance = 0.0;

    private double targetSpeedRps = 8;

    public ShotCalculatorState m_shotCalculatorState = ShotCalculatorState.IDLE;

    public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public enum ShotCalculatorState {
        IDLE,
        SOTM
    }

    @Override
    public void periodic() {
        if (m_shotCalculatorState == ShotCalculatorState.IDLE) {
            //something here later
        }
       else if (m_shotCalculatorState == ShotCalculatorState.SOTM) {
        Pose2d drivetrainPose = this.drivetrain.getPose();

        targetDistance = drivetrainPose.getTranslation().getDistance(getTargetLocation().toPose2d().getTranslation());
        targetSpeedRps = ShooterConstants.DISTANCE_TO_SHOT_SPEED.get(targetDistance);

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(ShooterConstants.BALL_TRANSFORM_CENTER);

        ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldRelativeSpeeds();
        ChassisAccelerations drivetrainAccelerations = drivetrain.getFieldRelativeAccelerations();

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose, getTargetLocation(),
                drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
                5, 0.01);

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
        }
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
    }

    public Pose3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }

    public boolean inAllianceZone() {
        Pose2d pose = drivetrain.getPose();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue
                && pose.getMeasureX()
                        .lt(Meters.of(FieldLayout.BLUE_ALLIANCE_ZONE)
                                .plus(Meters.of(DriveTrainConstants.ROBOT_DIMENSIONS).div(2)))
                || !isBlue && pose.getMeasureX()
                        .gt(Meters.of(FieldLayout.RED_ALLIANCE_ZONE)
                                .minus(Meters.of(DriveTrainConstants.ROBOT_DIMENSIONS).div(2)));
    }

    public Pose3d getTargetLocation() {
        Pose2d pose = drivetrain.getPose();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = pose.getMeasureY().gt(Meters.of(FieldLayout.FIELD_WIDTH).div(2));
        Pose3d bluePassingSpotLeft = new Pose3d(FieldLayout.BLUE_PASSING_SPOT_LEFT.getX(), FieldLayout.BLUE_PASSING_SPOT_LEFT.getY(), FieldLayout.BLUE_PASSING_SPOT_LEFT.getZ(), new Rotation3d()); 
        Pose3d bluePassingSpotRight = new Pose3d(FieldLayout.BLUE_PASSING_SPOT_RIGHT.getX(), FieldLayout.BLUE_PASSING_SPOT_RIGHT.getY(), FieldLayout.BLUE_PASSING_SPOT_RIGHT.getZ(), new Rotation3d());
        boolean onRedLeftSide = pose.getMeasureY().lt(Meters.of(FieldLayout.FIELD_WIDTH).div(2));
        Pose3d redPassingSpotLeft = new Pose3d(FieldLayout.RED_PASSING_SPOT_LEFT.getX(), FieldLayout.RED_PASSING_SPOT_LEFT.getY(), FieldLayout.RED_PASSING_SPOT_LEFT.getZ(), new Rotation3d()); 
        Pose3d redPassingSpotRight = new Pose3d(FieldLayout.RED_PASSING_SPOT_RIGHT.getX(), FieldLayout.RED_PASSING_SPOT_RIGHT.getY(), FieldLayout.RED_PASSING_SPOT_RIGHT.getZ(), new Rotation3d());

        if (inAllianceZone()) {
            if(isBlue) {
            targetLocation = FieldLayout.CENTER_OF_HUB_BLUE;
            }
            else {
                targetLocation = FieldLayout.CENTER_OF_HUB_RED;
            }
        }
        else {
            if(isBlue) {
                targetLocation = onBlueLeftSide ? bluePassingSpotLeft : bluePassingSpotRight;
            }
            else {
                targetLocation = onRedLeftSide ? redPassingSpotLeft : redPassingSpotRight;
            }
        }
        return targetLocation;
    }

    public double getAbsoluteAngleFromTargetSOTM() {
        // double[] botPose = getBotPose();
        // double rX = getBotPoseValue(botPose, 0);
        // double rY = getBotPoseValue(botPose, 1);
        double rX = this.drivetrain.getPose().getX();
        double rY = this.drivetrain.getPose().getY();

        return Math.toDegrees(
                Math.atan2(rY - currentEffectiveTargetPose.getY(), rX - currentEffectiveTargetPose.getX()))
                + 180;
    }

    public double getAbsouluteDistanceFromTargetSOTM() {
        double rX = this.drivetrain.getPose().getX();
        double rY = this.drivetrain.getPose().getY();
        // double tX;
        // double tAngle = getAbsoluteAngleFromTargetSOTM();
        // tX = currentEffectiveTargetPose.getX();

        // double adjacent = rX - tX;
        double distanceFromTarget = Math.hypot(rX - getTargetLocation().getX(), rY - getTargetLocation().getY()); // hypotenuse = adjacent /
                                                                                     // cos(angle)

        return distanceFromTarget;
    }

      public void setShotCalculatorState(ShotCalculatorState shotCalculatorState) {
    this.m_shotCalculatorState = shotCalculatorState;
  }

   
      public ShotCalculatorState getShotCalculatorState() {
    return m_shotCalculatorState;
  }
}
