// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;

public class Shooter extends SubsystemBase {
private final ShooterIO io;
private final ShooterInputs inputs = new ShooterInputs();

public ShooterState shooterState;
public double m_distanceFromHub;
public double m_distanceFromTargetSOTM;
public double m_distanceFromPass;
private ShooterState previousShooterState = ShooterState.IDLE;

// private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
// private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
// private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

// private final SysIdRoutine shooterSysIdRoutine;

public enum ShooterState {
  IDLE,
  SHOOTING, 
  PASSING,
  SOTM,
  TESTING,
  MANUAL_CLOSE,
  MANUAL_FAR
}

  public Shooter(ShooterIO io) {
  this.io = io;
  shooterState = ShooterState.IDLE;

  // shooterSysIdRoutine = new SysIdRoutine(
  //     new SysIdRoutine.Config(),
  //     new SysIdRoutine.Mechanism(
  //         (Voltage volts) -> setKickerVolts(volts.in(Volts)),
  //         log -> {
  //           log.motor("kick-flywheel")
  //               .voltage(sysidAppliedVoltageMeasure.mut_replace(inputs.kickerMotorVoltage,
  //                   Volts))
  //               .angularPosition(sysidPositionMeasure
  //                   .mut_replace(inputs.kickerPosition, Rotations))
  //               .angularVelocity(
  //                   sysidVelocityMeasure.mut_replace(inputs.kickerMotorVelocityRPS,
  //                       RotationsPerSecond));
  //         },
  //         this));

}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (previousShooterState != shooterState) {
      if (shooterState == ShooterState.IDLE) {
        stopShooter();
      }
      previousShooterState = shooterState;
    }
    else if (shooterState == ShooterState.SHOOTING) {
      setShooterVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHub));
      setKickerVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHub) * (ShooterConstants.KICKER_TO_SHOOTER_RATIO));
    }
    else if (shooterState == ShooterState.SOTM) {
      setShooterVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromTargetSOTM));
      setKickerVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromTargetSOTM) * (ShooterConstants.KICKER_TO_SHOOTER_RATIO));
    }
    else if (shooterState == ShooterState.TESTING) {
      setShooterVolts(ShooterConstants.TESTING_STATE_VOLTS);
      setKickerVolts(ShooterConstants.TESTING_STATE_VOLTS);
    }
    else if (shooterState == ShooterState.MANUAL_CLOSE) {
      setShooterVelocity(ShooterConstants.MANUAL_SHOOT_CLOSE);
      setKickerVelocity(ShooterConstants.MANUAL_SHOOT_CLOSE * ShooterConstants.KICKER_TO_SHOOTER_RATIO);
    }
    else if (shooterState == ShooterState.MANUAL_FAR) {
      setShooterVelocity(ShooterConstants.MANUAL_SHOOT_FAR);
      setKickerVelocity(ShooterConstants.MANUAL_SHOOT_FAR * ShooterConstants.KICKER_TO_SHOOTER_RATIO);
    }

  //Logging
  //  DogLog.log("Shooter/Motor1/isConnected", inputs.shootMotor1Connected);
  //  DogLog.log("Shooter/Motor1/ReferenceVelocity", inputs.shootMotor1ReferenceVelocityRPS);
  //  DogLog.log("Shooter/Motor1/ClosedLoopReference", inputs.shootMotor1ClosedLoopReferenceRPS);
  //  DogLog.log("Shooter/Motor1/Voltage", inputs.shootMotor1Voltage);
  //  DogLog.log("Shooter/Motor1/TorqueCurrentAmps", inputs.shootMotor1TorqueCurrentAmps);
   SmartDashboard.putNumber("Shooter/Motor1/Velocity", inputs.shootMotor1VelocityRPS);
  //  DogLog.log("Shooter/Motor1/SupplyCurrentAmps", inputs.shootMotor1SupplyCurrentAmps);
  //  DogLog.log("Shooter/Motor1/TempCelsius", inputs.shootMotor1TempCelsius);
  //  DogLog.log("Shooter/Motor2/isConnected", inputs.shootMotor2Connected);
   SmartDashboard.putNumber("Shooter/Motor2/Velocity", inputs.shootMotor2VelocityRPS);
  //  DogLog.log("Shooter/Motor2/ReferenceVelocity", inputs.shootMotor2ReferenceVelocityRPS);
  //  DogLog.log("Shooter/Motor2/ClosedLoopReference", inputs.shootMotor2ClosedLoopReferenceRPS);
  //  DogLog.log("Shooter/Motor2/Voltage", inputs.shootMotor2Voltage);
  //  DogLog.log("Shooter/Motor2/TorqueCurrentAmps", inputs.shootMotor2TorqueCurrentAmps);
  //  DogLog.log("Shooter/Motor2/SupplyCurrentAmps", inputs.shootMotor2SupplyCurrentAmps);
  //  DogLog.log("Shooter/Motor2/TempCelsius", inputs.shootMotor2TempCelsius);
   SmartDashboard.putNumber("Shooter/Kicker/Velocity", inputs.kickerMotorVelocityRPS);

  }

  public void setShooterVelocity(double shootRPS) {
    io.setShooterVelocity(shootRPS);
  }
 
  public void setShooterVolts(double shootMotorVolts) {
    io.setShooterVolts(shootMotorVolts);
  }

  public void setKickerVolts(double kickerVolts) {
    io.setKickerVolts(kickerVolts);
  }

  public void setKickerVelocity(double kickerRPS) {
    io.setKickerVelocity(kickerRPS);
  }

  public void stopShooter() {
    io.stopShooter();
  }  

  public ShooterState getShooterState() {
    return shooterState;
  }

  public void setShooterState(ShooterState shooterState) {
    this.shooterState = shooterState;
  }

  public void setDistanceFromHub(double distanceFromHub) {
    m_distanceFromHub = distanceFromHub;
  }

  public void setDistanceFromTargetSOTM(double distanceFromHubSOTM) {
    m_distanceFromTargetSOTM = distanceFromHubSOTM;
  }

  public void setDistanceFromPass(double distanceFromPass) {
    m_distanceFromPass = distanceFromPass;
  }

  public boolean isAtSpeed(double toleranceRPS) {
    double targetRPS;
    double targetRPSKicker;

    if (shooterState == ShooterState.SHOOTING) {
      targetRPS = ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHub);
      targetRPSKicker = ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHub) * ShooterConstants.KICKER_TO_SHOOTER_RATIO;
    } else if (shooterState == ShooterState.SOTM) {
      targetRPS = ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromTargetSOTM);
      targetRPSKicker = ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromTargetSOTM) * ShooterConstants.KICKER_TO_SHOOTER_RATIO;
    } else if (shooterState == ShooterState.MANUAL_CLOSE) {
      targetRPS = ShooterConstants.MANUAL_SHOOT_CLOSE;
      targetRPSKicker = ShooterConstants.MANUAL_SHOOT_CLOSE * ShooterConstants.KICKER_TO_SHOOTER_RATIO;
    } else if (shooterState == ShooterState.MANUAL_FAR) {
      targetRPS = ShooterConstants.MANUAL_SHOOT_FAR;
      targetRPSKicker = ShooterConstants.MANUAL_SHOOT_FAR * ShooterConstants.KICKER_TO_SHOOTER_RATIO;
    } else {
      return false; // IDLE or TESTING — not trying to hold a velocity
    }
    boolean isAtSpeedShooter = Math.abs(inputs.shootMotor1VelocityRPS - targetRPS) < toleranceRPS;
    boolean isAtSpeedKicker = Math.abs(inputs.kickerMotorVelocityRPS - targetRPSKicker) < toleranceRPS;
    SmartDashboard.putNumber("Shooter/TargetRPS", targetRPS);
    SmartDashboard.putBoolean("Shooter/isAtSpeed", isAtSpeedShooter);

    return isAtSpeedShooter && isAtSpeedKicker;
  }

  //  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //       return shooterSysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
  //   }

  //   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //       return shooterSysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
  //   }
}