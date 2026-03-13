// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.shotlib.ShotCalculator;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.vision.LimelightHelpers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIOKraken;

public class RobotContainer {
    // subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake;
    public final Roller roller;
    public final Shooter shooter;
    public final Hood hood;
    public final Superstructure superstructure;
    public final ShotCalculator shotcalculator;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveTrainConstants.ROBOT_MAX_SPEED * 0.1).withRotationalDeadband(DriveTrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DriveTrainConstants.ROBOT_MAX_SPEED);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController testJoystick = new CommandXboxController(2);


    public RobotContainer() {        
        intake = new Intake(new IntakeIOKraken());
        roller = new Roller(new RollerIOKraken());
        shooter = new Shooter(new ShooterIOKraken());
        hood = new Hood(new HoodIOKraken());
        shotcalculator = new ShotCalculator(drivetrain);
        superstructure = new Superstructure(drivetrain, intake, roller, shooter, hood, shotcalculator);


        
        configureBindings();
        // drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(calculateVelocity(joystick.getLeftY())) // Drive forward with negative Y (forward)
                    .withVelocityY(calculateVelocity(joystick.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(calculateRotationalVelocity()) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // DRIVER CONTROLLER

        // sets robot shoot on/off while right trigger is held
        (joystick.rightTrigger().and(superstructure::pointedAtTarget))
                .whileTrue(shootingCommand());

        // manual shooting
        joystick.rightBumper().whileTrue(superstructure.shootManually());
        joystick.rightBumper().onFalse(superstructure.stopShooting());

        // sets intake on/off on toggle default = on
        joystick.leftBumper().onTrue(superstructure.deployIntakingCommand());
        // retracts intake
        joystick.leftTrigger().onTrue(superstructure.retractIntake());

        // sets hood angle for a close shot and far shot
        joystick.a().onTrue(superstructure.setHoodAngleClose());
        joystick.y().onTrue(superstructure.setHoodAngleFar());

        // TESTING JOYSTICK

        
        // intake deploy and retract
        testJoystick.leftBumper().whileTrue(superstructure.testIntakeDeployDown());
        testJoystick.leftBumper().onFalse(superstructure.retractIntake());

        testJoystick.leftTrigger().whileTrue(superstructure.testIntakeDeployUp());
        testJoystick.leftTrigger().onFalse(superstructure.retractIntake());

        // intake rollers test
        testJoystick.a().whileTrue(superstructure.testIntakeRollers());
        testJoystick.a().onFalse(superstructure.stopIntakeRollers());

        // roller floor test
        testJoystick.rightBumper().whileTrue(superstructure.testRollers());
        testJoystick.rightBumper().onFalse(superstructure.stopRollers());

        // shooter test
        testJoystick.rightTrigger().whileTrue(superstructure.testShooter());
        testJoystick.rightTrigger().onFalse(superstructure.stopShooting());

        // hood test
        testJoystick.b().whileTrue(superstructure.testHood());
        testJoystick.b().onFalse(superstructure.stopHood());

        // // SYS ID TUNING
        // testJoystick.x().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // testJoystick.y().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        // testJoystick.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        // testJoystick.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));

        // testJoystick.x().whileTrue(intake.sysIdQuasistaticCommand(Direction.kForward));
        // testJoystick.y().whileTrue(intake.sysIdQuasistaticCommand(Direction.kReverse));
        // testJoystick.a().whileTrue(intake.sysIdDynamicCommand(Direction.kForward));
        // testJoystick.b().whileTrue(intake.sysIdDynamicCommand(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    public double calculateVelocity(double joystick) {
        double leftJoystick = MathUtil.applyDeadband(joystick, 0.1);
        double velocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftJoystick, 2.5);
        return velocity;
    }

    public double calculateRotationalVelocity() {
        double omega = 0;
        if (joystick.rightTrigger().getAsBoolean()) {
            omega = superstructure.turnToTargetSpeed();
        } else {
            double rightX = MathUtil.applyDeadband(joystick.getRightX(), 0.1);
            omega = -DriveTrainConstants.MAX_ANGULAR_RATE * scale(rightX, 2.5);
        }
        return omega;
    }

    private double scale(double in, double scale) {
        return Math.tan(in * Math.atan(scale)) / scale;
    }

    private boolean isJoystickNeutral() {
        return Math.hypot(joystick.getLeftX(), joystick.getLeftY()) < 0.1;
    }

    private Command shootingCommand() {
        return Commands.parallel(
                superstructure.startShooting(),
                Commands.run(() -> {
                    if (isJoystickNeutral()) {
                        drivetrain.setControl(brake);
                    } else {
                        drivetrain.setControl(
                                drive.withVelocityX(calculateVelocity(joystick.getLeftY()))
                                        .withVelocityY(calculateVelocity(joystick.getLeftX()))
                                        .withRotationalRate(calculateRotationalVelocity()));
                    }
                }, drivetrain));
    }

    public void resetGyro() {
        if(LimelightHelpers.getTV(LimelightConstants.SHOOTER_LIMELIGHT)) {
            drivetrain.resetGyro(LimelightConstants.SHOOTER_LIMELIGHT);
        }
        if(LimelightHelpers.getTV(LimelightConstants.LEFT_LIMELIGHT)) {
            drivetrain.resetGyro(LimelightConstants.LEFT_LIMELIGHT);
        }
    }
}
