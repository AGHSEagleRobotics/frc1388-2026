// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.System.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;

public class Shooter extends SubsystemBase {
private final ShooterIO io;
private final ShooterInputs inputs = new ShooterInputs();

public ShooterState shooterState;
public double m_distanceFromHub;
public double m_distanceFromHubSOTM;
public double m_distanceFromPass;

private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

private final SysIdRoutine shooterSysIdRoutine;

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

  shooterSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Voltage volts) -> setShooterVolts(volts.in(Volts)),
          log -> {
            log.motor("shooter-flywheel")
                .voltage(sysidAppliedVoltageMeasure.mut_replace(inputs.shootMotor1Voltage,
                    Volts))
                .angularPosition(sysidPositionMeasure
                    .mut_replace(inputs.shootMotor1Position, Rotations))
                .angularVelocity(
                    sysidVelocityMeasure.mut_replace(inputs.shootMotor1VelocityRPS,
                        RotationsPerSecond));
          },
          this));

}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (shooterState == ShooterState.IDLE) {
      stopShooter();
    }
    else if (shooterState == ShooterState.SHOOTING) {
      setShooterVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHub));
      setKickerVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHub) * (4/3));
    }
    else if (shooterState == ShooterState.PASSING) {
      setShooterVelocity(ShooterConstants.DISTANCE_TO_PASS_RPM.get(m_distanceFromPass));
      setKickerVelocity(ShooterConstants.DISTANCE_TO_PASS_RPM.get(m_distanceFromPass) * (4/3));
    }
    else if (shooterState == ShooterState.SOTM) {
      setShooterVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHubSOTM));
      setKickerVelocity(ShooterConstants.DISTANCE_TO_SHOT_RPM.get(m_distanceFromHubSOTM) * (4/3));
    }
    else if (shooterState == ShooterState.TESTING) {
      setShooterVolts(ShooterConstants.TESTING_STATE_VOLTS);
      setKickerVolts(ShooterConstants.TESTING_STATE_VOLTS);
    }
    else if (shooterState == ShooterState.MANUAL_CLOSE) {
      setShooterVelocity(ShooterConstants.MANUAL_SHOOT_CLOSE);
    }
    else if (shooterState == ShooterState.MANUAL_FAR) {
      setShooterVelocity(ShooterConstants.MANUAL_SHOOT_FAR);
    }

  //Logging
   SmartDashboard.putBoolean("Shooter/Motor1/isConnected", inputs.shootMotor1Connected);
   SmartDashboard.putNumber("Shooter/Motor1/Velocity", inputs.shootMotor1VelocityRPS);
   SmartDashboard.putNumber("Shooter/Motor1/ReferenceVelocity", inputs.shootMotor1ReferenceVelocityRPS);
   SmartDashboard.putNumber("Shooter/Motor1/ClosedLoopReference", inputs.shootMotor1ClosedLoopReferenceRPS);
   SmartDashboard.putNumber("Shooter/Motor1/Voltage", inputs.shootMotor1Voltage);
   SmartDashboard.putNumber("Shooter/Motor1/TorqueCurrentAmps", inputs.shootMotor1TorqueCurrentAmps);
   SmartDashboard.putNumber("Shooter/Motor1/SupplyCurrentAmps", inputs.shootMotor1SupplyCurrentAmps);
   SmartDashboard.putNumber("Shooter/Motor1/TempCelsius", inputs.shootMotor1TempCelsius);

   SmartDashboard.putBoolean("Shooter/Motor2/isConnected", inputs.shootMotor2Connected);
   SmartDashboard.putNumber("Shooter/Motor2/Velocity", inputs.shootMotor2VelocityRPS);
   SmartDashboard.putNumber("Shooter/Motor2/ReferenceVelocity", inputs.shootMotor2ReferenceVelocityRPS);
   SmartDashboard.putNumber("Shooter/Motor2/ClosedLoopReference", inputs.shootMotor2ClosedLoopReferenceRPS);
   SmartDashboard.putNumber("Shooter/Motor2/Voltage", inputs.shootMotor2Voltage);
   SmartDashboard.putNumber("Shooter/Motor2/TorqueCurrentAmps", inputs.shootMotor2TorqueCurrentAmps);
   SmartDashboard.putNumber("Shooter/Motor2/SupplyCurrentAmps", inputs.shootMotor2SupplyCurrentAmps);
   SmartDashboard.putNumber("Shooter/Motor2/TempCelsius", inputs.shootMotor2TempCelsius);

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

  public void setDistanceFromHubSOTM(double distanceFromHubSOTM) {
    m_distanceFromHubSOTM = distanceFromHubSOTM;
  }

  public void setDistanceFromPass(double distanceFromPass) {
    m_distanceFromPass = distanceFromPass;
  }

   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
    }
}