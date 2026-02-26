// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.lang.System.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;

public class Shooter extends SubsystemBase {
private final ShooterIO io;
private final ShooterInputs inputs = new ShooterInputs();

public ShooterState shooterState;

public enum ShooterState {
  IDLE,
  SHOOTING, 
  PASSING,
  SOTM,
}

private final SysIdRoutine shooterSysIdRoutine =
  new SysIdRoutine(
    new SysIdRoutine.Config(null, null, null) , 
    new SysIdRoutine.Mechanism(null, null, null));        

  public Shooter(ShooterIO io) {
  this.io = io;
  shooterState = ShooterState.IDLE;
  }

  @Override
  public void periodic() {
    if (shooterState == ShooterState.IDLE) {
      stopShooter();
    }
    else if (shooterState == ShooterState.SHOOTING) {
      setShooterVelocity(ShooterConstants.SHOOTING_STATE_VELOCITY);
    }
    else if (shooterState == ShooterState.PASSING) {
      setShooterVelocity(ShooterConstants.PASSING_STATE_VELOCITY);
    }
    else if (shooterState == ShooterState.SOTM) {
      setShooterVelocity(ShooterConstants.SOTM_STATE_VELOCITY);
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
    io.setShooterVelocity(0);
  }
 
  public void setShooterVolts(double shootMotorVolts) {
    io.setShooterVolts(0);
  }

  public void setKickerVolts(double kickerVolts) {
    io.setKickerVolts(0);
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
}