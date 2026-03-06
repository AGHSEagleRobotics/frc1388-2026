// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.vision.Dashboard;

public class AutoMethod extends SubsystemBase {
  private final Dashboard m_dashboard;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Intake m_intake;
  private final Roller m_roller;
  private final Shooter m_shooter;
  private final Hood m_hood;
  
  /** Creates a new AutoMethod. */
  public AutoMethod(Dashboard dashboard, CommandSwerveDrivetrain drivetrain, Intake intake, Roller roller, Shooter shooter, Hood hood) {
    m_dashboard = dashboard;
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_roller = roller;
    m_shooter = shooter;
    m_hood = hood;
  }

  public Command SitStillLookPretty(){
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_dashboard.getObjective().name();
    System.out.println(m_dashboard.getObjective().name());
  }
}
