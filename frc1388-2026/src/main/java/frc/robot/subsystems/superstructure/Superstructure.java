// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Set;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.IntakeConstants;
import frc.robot.shotlib.ShotCalculator;
import frc.robot.shotlib.ShotCalculator.ShotCalculatorState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.Roller.RollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {

  public final CommandSwerveDrivetrain m_driveTrain;
  public final Intake m_intake;
  public final Roller m_roller;
  public final Shooter m_shooter;
  public ShotCalculator m_shotCalculator;
  public RollerState m_rollerState;
  public ShooterState m_shooterState;
  public IntakeState m_intakeState;
  public boolean m_isAtSpeed;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public static final PIDController rotationPID = new PIDController(0.075, 0, 0);
  /** Creates a new Superstructure. */
  public Superstructure(CommandSwerveDrivetrain driveTrain, Intake intake, Roller roller, Shooter shooter, ShotCalculator shotCalculator) {
    m_driveTrain = driveTrain;
    m_intake = intake;
    m_roller = roller;
    m_shooter = shooter;
    m_shotCalculator = shotCalculator;

    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setTolerance(4);
    // rotationPID.setIZone(2);
    // rotationPID.setIntegratorRange(-0.36, 0.36);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isInShooterState()) {
    m_isAtSpeed = isAtSpeed();
    }
    
    if (m_shooter.getShooterState() == ShooterState.SHOOTING) {
      m_shooter.setDistanceFromHub(m_driveTrain.getAbsouluteDistanceFromHub());
    }
    if (m_shooter.getShooterState() == ShooterState.SOTM) {
      m_shooter.setDistanceFromTargetSOTM(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
    }
  }

  public Command startShooting() {
    m_intakeState = m_intake.getIntakeState();
    return Commands.run(() -> {
      if (!isRobotMoving()) {
        m_shooter.setShooterState(ShooterState.SHOOTING);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
        if (m_isAtSpeed) {
          m_roller.setRollerState(RollerState.SHOOTING);
          m_intake.setIntakeState(IntakeState.SHOOTING);
        }
      } else {
        m_shooter.setShooterState(ShooterState.SOTM);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.SOTM);
        m_intake.setIntakeState(IntakeState.INTAKING);
        if (m_isAtSpeed) {
          m_roller.setRollerState(RollerState.SHOOTING);
        }
      }
    },
        m_roller, m_shooter, m_intake, m_shotCalculator);
  }

  public Command stopShooting() {
    return Commands.runOnce(() -> {
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
      m_intake.setIntakeState(IntakeState.INTAKING);
      m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
      m_isAtSpeed = false;

    },
    m_roller, m_shooter, m_intake, m_shotCalculator);
  }

  public Command deployIntakingCommand() {
    m_intakeState = m_intake.getIntakeState();
    return Commands.defer(() ->
      Commands.run(() -> {
      if ((m_intake.getPosition() < IntakeConstants.POSITION_TOLERANCE)) {
        m_intake.setIntakeState(IntakeState.EXTENDED);
        m_roller.setRollerState(RollerState.IDLE);
      } else {
        m_intake.setIntakeState(IntakeState.INTAKING);
        m_roller.setRollerState(RollerState.INTAKING);
      }},
      m_roller,
      m_intake).until(() -> m_intake.getIntakeState() == IntakeState.INTAKING)
    , Set.of(m_roller, m_intake)
    
    );
    
  }

  public Command retractIntake() {
    return Commands.runOnce(() -> {
      m_intake.setIntakeState(IntakeState.RETRACT);
      m_roller.setRollerState(RollerState.IDLE);
    },
    m_roller, m_intake);
  }

  public Command shootManually() {
    return Commands.run(() -> {

      if (m_shooter.getShooterState() == ShooterState.MANUAL_FAR) {
        m_shooter.setShooterState(ShooterState.MANUAL_FAR);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
      } else {
        m_shooter.setShooterState(ShooterState.MANUAL_CLOSE);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
      }
      if (m_isAtSpeed) {
        m_intake.setIntakeState(IntakeState.SHOOTING);
        m_roller.setRollerState(RollerState.SHOOTING);
      }
    },
    m_roller, m_shooter, m_intake, m_shotCalculator).until(() -> m_roller.getRollerState() == RollerState.SHOOTING);
  }
  
  public Command testIntakeDeployDown() {
    return this.runOnce(() -> 
    m_intake.setIntakeState(IntakeState.TESTINGDEPLOYDOWN));
  }

  public Command testIntakeDeployUp() {
    return this.runOnce(() -> 
    m_intake.setIntakeState(IntakeState.TESTINGDEPLOYUP));
  }

  public Command testIntakeRollers() {
    return this.runOnce(() -> 
    m_intake.setIntakeState(IntakeState.TESTINGROLLER));
  }

  public Command stopIntakeRollers() {
    return this.runOnce(() ->
    m_intake.setIntakeState(IntakeState.STOP));
  }

  public Command testRollers() {
    return this.runOnce(() ->
    m_roller.setRollerState(RollerState.TESTING));
  }

  public Command stopRollers() {
    return this.runOnce(() -> 
    m_roller.setRollerState(RollerState.IDLE));
  }

  public Command testShooter() {
    return this.runOnce(() ->
    m_shooter.setShooterState(ShooterState.TESTING));
  }

  public Command outTake() {
    return Commands.runOnce(() -> {
      m_intake.setIntakeState(IntakeState.REVERSE);
      m_roller.setRollerState(RollerState.REVERSE);
    });
  }

  // public Command rotateToHub() {
  //   return Commands.run(() -> {
  //     m_driveTrain.setControl(
  //         new SwerveRequest.FieldCentric()
  //             .withVelocityX(0)
  //             .withVelocityY(0)
  //             .withRotationalRate(turnToTargetSpeed()));
  //   }, m_driveTrain)
  //       .until(() -> pointedAtTarget())
  //       .withName("rotateToHub");
  // }

  public Command startShootingAuto() {
    return Commands.runOnce(() -> {
      m_shooter.setShooterState(ShooterState.SHOOTING);
    })
        .andThen(Commands.run(() -> {
          m_driveTrain.setControl(
              new SwerveRequest.FieldCentric()
                  .withVelocityX(0)
                  .withVelocityY(0)
                  .withRotationalRate(turnToTargetSpeed()));
        }, m_driveTrain)
            .until(() -> pointedAtTarget())
            .withName("rotateToHub"))
        .andThen(Commands.run(() -> {
          m_driveTrain.applyRequest(() -> brake);
          m_driveTrain.setControl(
              new SwerveRequest.FieldCentric()
                  .withVelocityX(0)
                  .withVelocityY(0)
                  .withRotationalRate(0));
          if (m_isAtSpeed) {
            m_intake.setIntakeState(IntakeState.SHOOTING);
            m_roller.setRollerState(RollerState.SHOOTING);
          }
        }, m_driveTrain))
        .withName("startShootingAuto");
  }

  public double turnToTargetSpeed() {
    double angleFromTarget = m_driveTrain.getAbsoluteAngleFromHub();
    angleFromTarget = MathUtil.inputModulus(angleFromTarget, -180, 180);
    double rz = m_driveTrain.getState().Pose.getRotation().getDegrees();
    double speed = (rotationPID.calculate(rz, angleFromTarget));
    speed = MathUtil.clamp(speed, -DriveTrainConstants.MAX_ANGULAR_RATE, DriveTrainConstants.MAX_ANGULAR_RATE);

    return speed;
  }

  public boolean pointedAtTarget() {
    double angleFromTarget = m_driveTrain.getAbsoluteAngleFromHub();
    angleFromTarget = MathUtil.inputModulus(angleFromTarget, -180, 180);
    double rz = m_driveTrain.getState().Pose.getRotation().getDegrees();
    return Math.abs(rz - angleFromTarget) < 4;
  }

  public double turnToTargetSpeedSOTM() {
    double angleFromTarget = m_shotCalculator.getAbsoluteAngleFromTargetSOTM();
    angleFromTarget = MathUtil.inputModulus(angleFromTarget, -180, 180);
    double rz = m_driveTrain.getState().Pose.getRotation().getDegrees();
    double speed = rotationPID.calculate(rz, angleFromTarget);
    return MathUtil.clamp(speed, -DriveTrainConstants.MAX_ANGULAR_RATE, DriveTrainConstants.MAX_ANGULAR_RATE);
}

public boolean pointedAtTargetSOTM() {
    double angleFromTarget = m_shotCalculator.getAbsoluteAngleFromTargetSOTM();
    angleFromTarget = MathUtil.inputModulus(angleFromTarget, -180, 180);
    double rz = m_driveTrain.getState().Pose.getRotation().getDegrees();
    return Math.abs(rz - angleFromTarget) < 4;
}

  public boolean isAtSpeed() {
    return m_shooter.isAtSpeed(2);
  }

  public boolean isRobotMoving() {
    ChassisSpeeds speeds = m_driveTrain.getFieldRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return linearSpeed > 0.05;
  }

  public boolean isInShooterState() {
    if (m_shooter.getShooterState() == ShooterState.MANUAL_CLOSE
        || m_shooter.getShooterState() == ShooterState.MANUAL_FAR
        || m_shooter.getShooterState() == ShooterState.SHOOTING
        || m_shooter.getShooterState() == ShooterState.PASSING
        || m_shooter.getShooterState() == ShooterState.SOTM) {
      return true;
    }
    return false;
  }

  public boolean isBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public Command resetPositionOverBumpLeft() {
    return this.runOnce(() -> {
      if (isBlue()) {
        m_driveTrain.resetPose(new Pose2d(6.204, 5.880, new Rotation2d()));
      } else {
        m_driveTrain
            .resetPose(new Pose2d(FieldLayout.FIELD_LENGTH - 6.204, FieldLayout.FIELD_WIDTH - 5.880, new Rotation2d()));
      }
    });
  }

  public Command resetPositionOverBumpRight() {
    return this.runOnce(() -> {
      if (isBlue()) {
        m_driveTrain.resetPose(new Pose2d(6.204, 2.189, new Rotation2d()));
      } else {
        m_driveTrain
            .resetPose(new Pose2d(FieldLayout.FIELD_LENGTH - 6.204, FieldLayout.FIELD_WIDTH - 2.189, new Rotation2d()));
      }
    });
  }

}
