// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.Goal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.Roller;

public class Superstructure extends SubsystemBase {

  public final CommandSwerveDrivetrain m_driveTrain;
  public final Roller m_roller;

  /** Creates a new Superstructure. */
  public Superstructure(CommandSwerveDrivetrain driveTrain, Roller roller) {
    m_driveTrain = driveTrain;
    m_roller = roller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
