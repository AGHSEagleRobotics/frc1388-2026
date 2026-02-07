// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import java.util.function.Supplier;
import frc.robot.subsystems.Ale_ThrowerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Ale_ThrowerCommand extends Command {
  
  private final Ale_ThrowerSubsystem m_throwerSubsystem;
  private final Supplier<Double> m_Supplier;
  private final Double POWER_LIMIT = 0.75;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Ale_ThrowerCommand(Ale_ThrowerSubsystem throwerSubsystem, Supplier <Double> supplier) {
    m_throwerSubsystem = throwerSubsystem;
    m_Supplier = supplier; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_throwerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double supplier = -m_Supplier.get();
    m_throwerSubsystem.setPower(POWER_LIMIT*supplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
