// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

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
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.generated.TunerConstants;
import frc.robot.shotlib.ShotCalculator;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOKraken;
import frc.robot.subsystems.rollers.Roller.RollerState;
import frc.robot.subsystems.superstructure.Superstructure.RobotState;

public class RobotContainer {
    // subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake;
    public final Roller roller;
    public final Shooter shooter;
    public final Hood hood;
    public final Superstructure superstructure;
    public final ShotCalculator shotcalculator;
    public RobotState robotState;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double m_rotationalVelocity = 0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController testJoystick = new CommandXboxController(2);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    

    public RobotContainer() {        
        intake = new Intake(new IntakeIOKraken());
        roller = new Roller(new RollerIOKraken());
        shooter = new Shooter(new ShooterIOKraken());
        hood = new Hood(new HoodIOKraken());
        shotcalculator = new ShotCalculator(drivetrain);
        superstructure = new Superstructure(drivetrain, intake, roller, shooter, hood, shotcalculator);


        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        //register commands


        configureBindings();
        configureAutonomousCommands();

    //       private void configureAutonomousCommands() {
    //     Command strikeAStillPose = new InstantCommand(() -> System.out.println("This is kinda boring, no?"));

    //      Command moveYourBass = new SequentialCommandGroup(
    //         new DriveDistanceCommand(drive, .0),   // drive 2 units
    //         new TurnToAngleCommand(drive, 90)       // turn 90 degrees
    //     );


    //   }

        NamedCommands.registerCommand("startShooting", superstructure.startShooting());
        NamedCommands.registerCommand("stopShooting", superstructure.stopShooting());


    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        double leftX = MathUtil.applyDeadband(joystick.getLeftY(), 0.1);
        double leftY = MathUtil.applyDeadband(joystick.getLeftX(), 0.1);

        
        double xVelocity = -MaxSpeed * scale(leftX, 2.5);
        double yVelocity = -MaxSpeed * scale(leftY, 2.5);
        

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xVelocity) // Drive forward with negative Y (forward)
                    .withVelocityY(yVelocity) // Drive left with negative X (left)
                    .withRotationalRate(getRotationalVelocity()) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // DRIVER CONTROLLER

        // sets robot shoot on/off while right trigger is held
        if (superstructure.pointedAtTarget()) {
        joystick.rightTrigger().whileTrue(superstructure.startShooting());
        }
        joystick.rightTrigger().onFalse(superstructure.stopShooting());

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
        testJoystick.leftBumper().onTrue(superstructure.testIntakeDeploy());
        testJoystick.leftTrigger().onTrue(superstructure.retractIntake());

        // intake rollers test
        testJoystick.rightTrigger().whileTrue(superstructure.testIntakeRollers());
        testJoystick.rightTrigger().onFalse(superstructure.stopIntakeRollers());

        // roller floor test
        testJoystick.rightBumper().whileTrue(superstructure.testRollers());
        testJoystick.rightBumper().onFalse(superstructure.stopRollers());

        // shooter test
        testJoystick.rightTrigger().whileTrue(superstructure.testShooter());
        testJoystick.rightTrigger().onFalse(getAutonomousCommand());

        // SYS ID TUNING
        testJoystick.x().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        testJoystick.y().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        testJoystick.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        testJoystick.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));

        testJoystick.pov(0).whileTrue(hood.sysIdQuasistaticCommand(Direction.kForward));
        testJoystick.pov(90).whileTrue(hood.sysIdQuasistaticCommand(Direction.kReverse));
        testJoystick.pov(180).whileTrue(hood.sysIdDynamicCommand(Direction.kForward));
        testJoystick.pov(270).whileTrue(hood.sysIdDynamicCommand(Direction.kReverse));
    }

    

    public Command getAutonomousCommand() {
    //     System.out.println("before running");
    //     System.out.println("*");
    //     System.out.println("*");
    //     System.out.println("*");
    //     System.out.println("*");
    //     System.out.println("*");
    //     System.out.println("*");
    //     System.out.println("*");
    //     try{
    //         System.out.println("post try");
    //         // Load the path you want to follow using its name in the GUI
    //         PathPlannerPath testPath = PathPlannerPath.fromPathFile("Lachemann testpath");
            
    //         // Create a path following command using AutoBuilder. This will also trigger event markers.
    //         System.out.println("returning testpath");
    //         return AutoBuilder.followPath(testPath);
            
    //     } catch (Exception e) {
    //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    // }
    return autoChooser.getSelected();

         
    }

    public double getRotationalVelocity() {
        double rightX = MathUtil.applyDeadband(joystick.getRightX(), 0.1);
        double omega = -MaxAngularRate * scale(rightX, 2.5);
        if (joystick.rightTrigger().getAsBoolean()) {
            omega = superstructure.turnToTargetSpeed();
        }
        return omega;
    }

    private double scale(double in, double scale) {
        return Math.tan(in * Math.atan(scale)) / scale;
    }
}
