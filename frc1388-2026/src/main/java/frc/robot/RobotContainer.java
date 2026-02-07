// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.OperatorConstants.LEDConstants;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.Ale_ThrowerCommand;
// import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Ale_ThrowerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

import frc.robot.Constants.OperatorConstants;


public class RobotContainer {
    // private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);
    // private final CommandXboxController m_driverController = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

      final CANdle m_candle = new CANdle(LEDConstants.CANDLE_CANID);
      final LEDSubsystem m_ledSubsystem = new LEDSubsystem(m_candle);
      private final TalonFX m_throwerMotor1 = new TalonFX(1); 
      private final TalonFX m_throwerMotor2 = new TalonFX(2); 
      private final TalonFX m_hoodMotor = new TalonFX(3); 
      private final Ale_ThrowerSubsystem m_throwerSubsystem = new Ale_ThrowerSubsystem(m_throwerMotor1, m_throwerMotor2, m_hoodMotor);
      private final SparkFlex m_intakeMotor1 = new SparkFlex(1, null);
      private final SparkFlex m_intakeMotor2 = new SparkFlex(2, null);
      private final SparkMax m_rollerMotor = new SparkMax(3, null);
      private final DigitalInput m_intakeOutLS = new DigitalInput(1);
      private final DigitalInput m_intakeInLS = new DigitalInput(2);

    //   private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
        // new SparkFlex(1, MotorType.kBrushless),
        // new SparkFlex(2, MotorType.kBrushless),
        // new SparkMax(3, MotorType.kBrushless),
        // new DigitalInput(1),
        // new DigitalInput(2);

    //   private final IntakeCommand m_intakeCommand = new IntakeCommand();
        // new SparkFlex(1, MotorType.kBrushless),
        // new SparkFlex(2, MotorType.kBrushless),
        // new SparkMax(3, MotorType.kBrushless),
        // new DigitalInput(1),
        // new DigitalInput(2));


      


    private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);


Ale_ThrowerCommand m_throwerCommand; 


    public RobotContainer() {
       
            configureBindings();
            // m_intakeSubsystem.setDefaultCommand(m_intakeCommand);
        

        // configureBindings();

    }

    private void configureBindings() {
     
    
            
        
    
    //     // Note that X is defined as forward according to WPILib convention,
    //     // and Y is defined as to the left according to WPILib convention.
    //     drivetrain.setDefaultCommand(
    //         // Drivetrain will execute this command periodically
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    //                 .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //         )
    //     );

    //     // Idle while the robot is disabled. This ensures the configured
    //     // neutral mode is applied to the drive motors while disabled.
    //     final var idle = new SwerveRequest.Idle();
    //     RobotModeTriggers.disabled().whileTrue(
    //         drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    //     );

    //     joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //     joystick.b().whileTrue(drivetrain.applyRequest(() ->
    //         point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
    //     ));

    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //     joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //     joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //     joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //     // Reset the field-centric heading on left bumper press.
    //     joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    //     drivetrain.registerTelemetry(logger::telemeterize);
    
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // // Then slowly drive forward (away from us) for 5 seconds.
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(0.5)
            //         .withVelocityY(0)
            //         .withRotationalRate(0)
            // )
            // .withTimeout(5.0),
            // // Finally idle for the rest of auton
            // drivetrain.applyRequest(() -> idle)
        );
    }
}
