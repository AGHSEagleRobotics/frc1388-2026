// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shotlib.ShotCalculator;
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
  public RobotState robotState;
  public RollerState rollerState;
  public ShooterState shooterState;
  public HoodState hoodState;

  public static final PIDController rotationPID = new PIDController(0.01, 0, .0);

  public enum RobotState {
    IDLE,
    INTAKEDEPLOY,
    INTAKING,
    SHOOTING,
    PASSING,
    SOTM,
    TESTING,
    MANUAL_SHORT,
    MANUAL_FAR
  }

  /** Creates a new Superstructure. */
  public Superstructure(CommandSwerveDrivetrain driveTrain, Intake intake, Roller roller, Shooter shooter, Hood hood, ShotCalculator shotCalculator) {
    m_driveTrain = driveTrain;
    m_intake = intake;
    m_roller = roller;
    m_shooter = shooter;
    m_hood = hood;
    m_shotCalculator = shotCalculator;
    robotState = RobotState.IDLE;

    rotationPID.enableContinuousInput(0, 360);
    // rotationPID.setIZone(2);
    // rotationPID.setIntegratorRange(-0.36, 0.36);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooter.setDistanceFromHub(m_driveTrain.getAbsouluteDistanceFromHub());
    m_shooter.setDistanceFromHubSOTM(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
    m_shooter.setDistanceFromPass(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
    m_hood.setDistanceFromHub(m_driveTrain.getAbsouluteDistanceFromHub());
    m_hood.setDistanceFromHubSOTM(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
    m_hood.setDistanceFromPass(m_shotCalculator.getAbsouluteDistanceFromTargetSOTM());
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public Command setRobotState(RobotState robotState) {
    return this.runOnce(() ->
    this.robotState = robotState);
  }

  public Command startShooting() {
    return this.runOnce(() -> {
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.SHOOTING);
      m_hood.setHoodState(HoodState.SHOOTING);
    });
  }

  public Command stopShooting() {
    return this.runOnce(() -> {
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
      m_hood.setHoodState(HoodState.IDLE);
    });
  }

  public Command deployIntakingCommand() {
    if (m_intake.getIntakeState() == IntakeState.INTAKING) {
      return this.runOnce(() -> {
        m_intake.setIntakeState(IntakeState.EXTENDED);
      });
    }
    return this.runOnce(() -> {
      m_intake.setIntakeState(IntakeState.INTAKING);
    });
  }

  public Command retractIntake() {
    return this.runOnce(() -> {
      m_intake.setIntakeState(IntakeState.RETRACT);
    });
  }

  public Command shootManually() {
    if (m_hood.getHoodState() == HoodState.MANUAL_CLOSE) {
      return this.runOnce(() -> {
        m_roller.setRollerState(RollerState.SHOOTING);
        m_shooter.setShooterState(ShooterState.MANUAL_CLOSE);
      });
    }
    else {
      return this.runOnce(() -> {
        m_roller.setRollerState(RollerState.SHOOTING);
        m_shooter.setShooterState(ShooterState.MANUAL_FAR);
      });
    }
  }

  public Command setHoodAngleClose() {
    return this.runOnce(() -> 
    m_hood.setHoodState(HoodState.MANUAL_CLOSE));
  }

  public Command setHoodAngleFar() {
    return this.runOnce(() -> 
    m_hood.setHoodState(HoodState.MANUAL_FAR));
  }
  
  public Command testIntakeDeploy() {
    return this.runOnce(() -> 
    m_intake.setIntakeState(IntakeState.EXTENDED));
  }

  public Command testIntakeRollers() {
    return this.runOnce(() -> 
    m_intake.setIntakeState(IntakeState.TESTING));
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
}
