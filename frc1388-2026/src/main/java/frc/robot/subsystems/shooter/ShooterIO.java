// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO {
 public static class ShooterInputs {
  //VelocityRPS is the actual speed of the motor in revolutions per second,
  //ReferenceVelocityRPS is the velocity that we are asking for it to be, 
  //and ClosedLoopReferenceRPS is the velocity that the motor is trying to reach at that moment
  public boolean shootMotor1Connected = true;
  public boolean shootMotor2Connected = true;

  public double shootMotor1VelocityRPS = 0.0;
  public double shootMotor2VelocityRPS = 0.0;

  public double shootMotor1ReferenceVelocityRPS = 0.0;
  public double shootMotor2ReferenceVelocityRPS = 0.0;

  public double shootMotor1ClosedLoopReferenceRPS = 0.0;
  public double shootMotor2ClosedLoopReferenceRPS = 0.0;

  public double shootMotor1Voltage = 0.0;
  public double shootMotor2Voltage = 0.0;

  public double shootMotor1TorqueCurrentAmps = 0.0;
  public double shootMotor2TorqueCurrentAmps = 0.0;
  
  public double shootMotor1SupplyCurrentAmps = 0.0;
  public double shootMotor2SupplyCurrentAmps = 0.0;

  public double shootMotor1TempCelsius = 0.0;
  public double shootMotor2TempCelsius = 0.0;

 }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setShooterVelocity(double motor1RPS, double motor2RPS) {}
    
  public default void setShooterVolts(double motor1Volts, double motor2Volts) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void stopShooter() {}
    
  public default void setCoastMode(boolean coast) {}
  
  }

