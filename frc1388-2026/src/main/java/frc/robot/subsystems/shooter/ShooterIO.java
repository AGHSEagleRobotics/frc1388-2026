// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO {
 public static class ShooterIOInputs {
  public double shootMotor1VelocityRPS = 0.0;
  public double shootMotor2VelocityRPS = 0.0;

  public double shootMotor1Voltage = 0.0;
  public double shootMotor2Voltage = 0.0;

  public double shootMotor1CurrentAmps = 0.0;
  public double shootMotor2CurrentAmps = 0.0;

  public double shootMotor1TempCelsius = 0.0;
  public double shootMotor2TempCelsius = 0.0;
 }

  default void updateInputs(ShooterIOInputs shooterIOInputs) {}

  public default void setVelocityMotor1(double rps) {}
  
  public default void setVelocityMotor2(double rps) {}
  
  public default void setVoltsShooter1(double volts) {}
  
  public default void setVoltsShooter2(double volts) {}
  
  public default void setCoastMode(boolean coast) {}
  
  }

