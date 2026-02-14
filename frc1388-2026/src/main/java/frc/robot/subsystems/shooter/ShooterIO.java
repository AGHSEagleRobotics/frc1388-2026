// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO {
  public default void setVoltsShooter1(double volts) {}
  
  public default void setVoltsShooter2(double volts) {}
  
  public default void setShooter1RPM(double rpm) {}
  
  public default void setShooter2RPM(double rpm) {}

  public default void getShooterRPM(double rpm) {}

  public default void setCoastMode(boolean coast) {}

  public default void getEncoderAngle(int radians) {}
  
  }

