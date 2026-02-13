// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO {
  public default void setVolts(double volts) {}
  
  public default void setCoastMode(boolean coast) {}
  
  public default void setShooterVelocity(double rpm) {}

  
  
  }

