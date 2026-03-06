// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

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
    
    if (robotState == RobotState.IDLE) {
      m_intake.setIntakeState(IntakeState.RETRACT);
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
      m_hood.setHoodState(HoodState.IDLE);
    } else if (robotState == RobotState.INTAKEDEPLOY) {
      m_intake.setIntakeState(IntakeState.EXTENDED);
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
      m_hood.setHoodState(HoodState.IDLE);
    } else if (robotState == RobotState.INTAKING) {
      m_intake.setIntakeState(IntakeState.INTAKING);
      m_roller.setRollerState(RollerState.INTAKING);
      m_shooter.setShooterState(ShooterState.IDLE);
      m_hood.setHoodState(HoodState.IDLE);
    } else if (robotState == RobotState.PASSING) {
      m_intake.setIntakeState(IntakeState.SHOOTING);
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.PASSING);
      m_hood.setHoodState(HoodState.PASSING);
    } else if (robotState == RobotState.SHOOTING) {
      m_intake.setIntakeState(IntakeState.SHOOTING);
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.SHOOTING);
      m_hood.setHoodState(HoodState.SHOOTING);
    } else if (robotState == RobotState.SOTM) {
      m_intake.setIntakeState(IntakeState.INTAKING);
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.SOTM);
      m_hood.setHoodState(HoodState.SOTM);
    } else if (robotState == RobotState.TESTING) {
      m_intake.setIntakeState(IntakeState.TESTING);
      m_roller.setRollerState(RollerState.TESTING);
      m_shooter.setShooterState(ShooterState.TESTING);
      m_hood.setHoodState(HoodState.TESTING);
    } else if (robotState == RobotState.MANUAL_SHORT) {
      m_intake.setIntakeState(IntakeState.SHOOTING);
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.MANUAL);
      m_hood.setHoodState(HoodState.MANUAL_SHORT);
    } else if (robotState == RobotState.MANUAL_FAR) {
      m_intake.setIntakeState(IntakeState.SHOOTING);
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.MANUAL);
      m_hood.setHoodState(HoodState.MANUAL_FAR);
    }
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
}
