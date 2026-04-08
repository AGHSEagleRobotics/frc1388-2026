// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Dashboard;
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

    public final Superstructure superstructure;
    public final ShotCalculator shotcalculator;
    public final Dashboard dashboard;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()

            // NOTE deadband changed to 0.01 from 0.1
            .withDeadband(DriveTrainConstants.ROBOT_MAX_SPEED * 0.01).withRotationalDeadband(DriveTrainConstants.MAX_ANGULAR_RATE * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();


    private final Telemetry logger = new Telemetry(DriveTrainConstants.ROBOT_MAX_SPEED);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // private final CommandXboxController testJoystick = new CommandXboxController(2);

    private SendableChooser<Command> autoChooser;


    public RobotContainer() {
        intake = new Intake(new IntakeIOKraken());
        roller = new Roller(new RollerIOKraken());
        shooter = new Shooter(new ShooterIOKraken());

        shotcalculator = new ShotCalculator(drivetrain);

        superstructure = new Superstructure(drivetrain, intake, roller, shooter, shotcalculator);
        // dashboard = new Dashboard();
        dashboard = new Dashboard(shooter, intake);

        NamedCommands.registerCommand("startShooting", superstructure.startShooting());

        NamedCommands.registerCommand("stopShooting", superstructure.stopShooting());

        NamedCommands.registerCommand("deployIntaking", superstructure.deployIntakingCommand());

        // NamedCommands.registerCommand("retractIntake", superstructure.retractIntake());
        NamedCommands.registerCommand("shootManually", superstructure.shootManually());

        NamedCommands.registerCommand("startShootingAuto", superstructure.startShootingAuto());

        // NamedCommands.registerCommand("stopIntakeRollers", superstructure.stopIntakeRollers());
        // NamedCommands.registerCommand("stopRollers", superstructure.stopRollers());
        
        drivetrain.configureAutoBuilder();

        
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        // register commands

    }

    private void configureAutonomousCommands() {
    // Command strikeAStillPose = new InstantCommand(() -> System.out.println("This
    // is kinda boring, no?"));

    // Command moveYourBass = new SequentialCommandGroup(
    // new DriveDistanceCommand(drive, .0), // drive 2 units
    // new TurnToAngleCommand(drive, 90) // turn 90 degrees
    // );

    }
//I'm putting this comment here to make it push maybe somehow 
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


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.

        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // DRIVER CONTROLLER

        // sets robot shoot on/off while right trigger is held

        (joystick.rightTrigger().and(superstructure::pointedAtTarget))
                .whileTrue(shootingCommand());
        joystick.rightTrigger().onFalse(superstructure.stopShooting());
        // manual shooting
        joystick.rightBumper().whileTrue(superstructure.shootManually());
        joystick.rightBumper().onFalse(superstructure.stopShooting());

        // sets intake on/off on toggle default = on
        joystick.leftBumper().onTrue(superstructure.deployIntakingCommand());
        // retracts intake
        joystick.leftTrigger().onTrue(superstructure.retractIntake());

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.a().whileTrue(superstructure.outTake());
        joystick.a().onFalse(superstructure.deployIntakingCommand());

        joystick.back().onTrue(new InstantCommand(() -> drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d()))));

        // TESTING JOYSTICK

        // intake deploy and retract
    
        // testJoystick.leftBumper().whileTrue(superstructure.testIntakeDeployDown());
        // testJoystick.leftBumper().onFalse(superstructure.retractIntake());

        // testJoystick.leftTrigger().whileTrue(superstructure.testIntakeDeployUp());
        // testJoystick.leftTrigger().onFalse(superstructure.retractIntake());

        // // intake rollers test
        // testJoystick.a().whileTrue(superstructure.testIntakeRollers());
        // testJoystick.a().onFalse(superstructure.stopIntakeRollers());

        // // roller floor test
        // testJoystick.rightBumper().whileTrue(superstructure.testRollers());
        // testJoystick.rightBumper().onFalse(superstructure.stopRollers());

        // // shooter test
        // testJoystick.rightTrigger().whileTrue(superstructure.testShooter());
        // testJoystick.rightTrigger().onFalse(superstructure.stopShooting());

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
        

        return autoChooser.getSelected();

    }

    public double calculateVelocity(double joystick) {
        double leftJoystick = MathUtil.applyDeadband(joystick, 0.1);
        double velocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftJoystick, 2.5);
        return velocity;
    }

    public double calculateRotationalVelocity() {
        double omega = 0;
        if (joystick.rightTrigger().getAsBoolean()) {
            if (!superstructure.pointedAtTarget()) {
                omega = superstructure.turnToTargetSpeed();
            }
        } else {
            double rightX = MathUtil.applyDeadband(joystick.getRightX(), 0.05);
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
    }
}
