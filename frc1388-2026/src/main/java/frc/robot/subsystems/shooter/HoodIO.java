// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface HoodIO {

  public static class HoodIOInputs{
    public double hoodMotorVelocityRPS = 0.0;

    public double hoodMotorVoltage = 0.0;

    public double hoodMotorCurrentAmps = 0.0;

    public double hoodMotorTempCelsius = 0.0;

  }
  default void updateInputs(HoodIOInputs hoodIOInputs){

    public default void 
  }


}
