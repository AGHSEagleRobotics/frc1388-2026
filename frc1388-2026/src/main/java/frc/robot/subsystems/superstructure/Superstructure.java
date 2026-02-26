// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.ironmaple.simulation.Goal;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.Roller.RollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {

  public final CommandSwerveDrivetrain m_driveTrain;
  public final Roller m_roller;
  public final Shooter m_shooter;
  public RobotState robotState;
  public RollerState rollerState;
  public ShooterState shooterState;

  enum RobotState {
    IDLE,
    INTAKEDEPLOY,
    INTAKING,
    SHOOTING,
    PASSING,
    SOTM,
  }

  /** Creates a new Superstructure. */
  public Superstructure(CommandSwerveDrivetrain driveTrain, Roller roller, Shooter shooter) {
    m_driveTrain = driveTrain;
    m_roller = roller;
    m_shooter = shooter;
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    if(robotState == RobotState.IDLE) {
      //add intake here
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
    }
    else if (robotState == RobotState.INTAKEDEPLOY) {
      //add intake here
      m_roller.setRollerState(RollerState.IDLE);
      m_shooter.setShooterState(ShooterState.IDLE);
    }
    else if(robotState == RobotState.INTAKING) {
      // add intake here
      m_roller.setRollerState(RollerState.INTAKING);
      m_shooter.setShooterState(ShooterState.IDLE);
    }
    else if (robotState == RobotState.PASSING) {
      // add intake here
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.PASSING);
    }
    else if (robotState == RobotState.SHOOTING) {
      // add intake here
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.SHOOTING);
    }
    else if (robotState == RobotState.SOTM) {
      //add intake here
      m_roller.setRollerState(RollerState.SHOOTING);
      m_shooter.setShooterState(ShooterState.SOTM);
    }
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
  }
}
