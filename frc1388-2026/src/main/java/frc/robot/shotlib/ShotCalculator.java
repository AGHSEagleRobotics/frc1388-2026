package frc.robot.shotlib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.ShooterConstants;
import frc.robot.shotlib.ShootOnTheFlyCalculator.InterceptSolution;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShotCalculator extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    private Pose3d targetLocation = FieldLayout.CENTER_OF_HUB_BLUE;

    private double targetDistance = 0.0;

    private double targetSpeedRps = 8;

    public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = drivetrain.getPose();

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        targetSpeedRps = ShooterConstants.DISTANCE_TO_SHOT_SPEED.get(targetDistance);

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(ShooterConstants.BALL_TRANSFORM_CENTER);

        ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldRelativeSpeeds();
        ChassisAccelerations drivetrainAccelerations = drivetrain.getFieldRelativeAccelerations();

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose, targetLocation,
                drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
                5, 0.01);

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
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
}
