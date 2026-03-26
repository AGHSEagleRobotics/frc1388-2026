// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  private final Timer m_timer = new Timer();
  private final double kMatchDuration = 120.0;
  private final ShuffleboardTab m_shuffleboardTab;
    private final static String SHUFFLEBOARD_TAB_NAME = "Competition";

  /** Creates a new Dashboard. */
    public Dashboard() {
      m_shuffleboardTab = Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
      Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);
      m_timer.start();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elapsedTime = m_timer.get();
    double remainingTime = kMatchDuration - elapsedTime;
    SmartDashboard.putNumber("Countdown Timer (s)", remainingTime);
  }
}
