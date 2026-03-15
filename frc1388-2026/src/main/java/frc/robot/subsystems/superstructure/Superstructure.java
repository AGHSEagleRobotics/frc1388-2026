// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.shotlib.ShotCalculator;
import frc.robot.shotlib.ShotCalculator.ShotCalculatorState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.Roller.RollerState;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Hood.HoodState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {

  public final CommandSwerveDrivetrain m_driveTrain;
  public final Intake m_intake;
  public final Roller m_roller;
  public final Shooter m_shooter;
  public final Hood m_hood;
  public ShotCalculator m_shotCalculator;
  public RollerState m_rollerState;
  public ShooterState m_shooterState;
  public HoodState m_hoodState;
  public IntakeState m_intakeState;
  public boolean m_isAtSpeed;

  public static final PIDController rotationPID = new PIDController(0.2, 0, .0);
  /** Creates a new Superstructure. */
  public Superstructure(CommandSwerveDrivetrain driveTrain, Intake intake, Roller roller, Shooter shooter, Hood hood, ShotCalculator shotCalculator) {
    m_driveTrain = driveTrain;
    m_intake = intake;
    m_roller = roller;
    m_shooter = shooter;
    m_hood = hood;
    m_shotCalculator = shotCalculator;

    rotationPID.enableContinuousInput(0, 360);
    rotationPID.setTolerance(2);;
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
      m_hood.setDistanceFromHub(m_driveTrain.getAbsouluteDistanceFromHub());
    }
    if (m_shooter.getShooterState() == ShooterState.PASSING) {
      m_shooter.setDistanceFromPass(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
      m_hood.setDistanceFromPass(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
    }
    if (m_shooter.getShooterState() == ShooterState.SOTM) {
      m_shooter.setDistanceFromHubSOTM(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
      m_hood.setDistanceFromHubSOTM(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
    }
  }

  public Command startShooting() {
    m_intakeState = m_intake.getIntakeState();
    return this.run(() -> {
      if (isRobotMoving()) {
        m_shooter.setShooterState(ShooterState.SOTM);
        m_hood.setHoodState(HoodState.SOTM);
        m_intake.setIntakeState(IntakeState.INTAKING);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.SOTM);
      } else {
        m_shooter.setShooterState(ShooterState.SHOOTING);
        m_hood.setHoodState(HoodState.SHOOTING);
        m_intake.setIntakeState(IntakeState.SHOOTING);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
      }
      if (m_isAtSpeed) {
      m_roller.setRollerState(RollerState.SHOOTING);
      }
    });
  }

  public Command stopShooting() {
    return this.runOnce(() -> {
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
      m_hood.setHoodState(m_hoodState);
      m_intake.setIntakeState(IntakeState.INTAKING);
      m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
      m_isAtSpeed = false;

    });
  }

  public Command deployIntakingCommand() {
    m_intakeState = m_intake.getIntakeState();
    return this.run(() -> {
      if ((m_intake.getIntakeState() == IntakeState.INTAKING) || (m_intake.getPosition() < IntakeConstants.POSITION_TOLERANCE)) {
        m_intake.setIntakeState(IntakeState.EXTENDED);
        m_roller.setRollerState(RollerState.IDLE);
      } else {
        m_intake.setIntakeState(IntakeState.INTAKING);
        m_roller.setRollerState(RollerState.INTAKING);
      }
    }).until(() -> m_intake.getIntakeState() == IntakeState.INTAKING);
  }

  public Command retractIntake() {
    return this.runOnce(() -> {
      m_intake.setIntakeState(IntakeState.RETRACT);
      m_roller.setRollerState(RollerState.IDLE);
    });
  }

  public Command shootManually() {
    m_hoodState = m_hood.getHoodState();
    return this.run(() -> {
      if (m_shooter.getShooterState() == ShooterState.MANUAL_FAR) {
        // m_shooter.setShooterState(ShooterState.MANUAL_FAR);
        m_hood.setHoodState(HoodState.MANUAL_FAR);
        m_intake.setIntakeState(IntakeState.SHOOTING);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);

      } else if (m_shooter.getShooterState() == ShooterState.MANUAL_CLOSE) {
        // m_shooter.setShooterState(ShooterState.MANUAL_CLOSE);
        m_hood.setHoodState(HoodState.MANUAL_CLOSE);
        m_intake.setIntakeState(IntakeState.SHOOTING);
        m_shotCalculator.setShotCalculatorState(ShotCalculatorState.IDLE);
      }
      if (m_isAtSpeed) {
        m_roller.setRollerState(RollerState.SHOOTING);
        }
    }).until(() -> m_roller.getRollerState() == RollerState.SHOOTING);
  }

  public Command setHoodAngleClose() {
    return this.runOnce(() -> 
    m_hood.setHoodState(HoodState.MANUAL_CLOSE));
  }

  public Command setHoodAngleFar() {
    return this.runOnce(() -> 
    m_hood.setHoodState(HoodState.MANUAL_FAR));
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

  public Command testHood() {
    return this.runOnce(() ->
    m_hood.setHoodState(HoodState.TESTING));
  }

  public Command stopHood() {
    return this.runOnce(() -> m_hood.setHoodState(HoodState.IDLE));
  }

  public double turnToTargetSpeed() {
    double angleFromTarget = m_shotCalculator.getAbsoluteAngleFromTargetSOTM();
    double rz = m_driveTrain.getAngle();
    rz = rz < 0 ? rz + 360 : rz;
    double speed = -(rotationPID.calculate(angleFromTarget - rz));
    return speed;
  }

  public boolean pointedAtTarget() {
    return rotationPID.atSetpoint();
  }

  public boolean isAtSpeed() {
    return m_shooter.isAtSpeed(6.5);
  }

  public boolean isRobotMoving() {
    ChassisSpeeds speeds = m_driveTrain.getFieldRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return linearSpeed > 0.01;
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
}
