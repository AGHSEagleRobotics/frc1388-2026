// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  public Dashboard() {
    
class Dashboard extends SubsystemBase {
  private final ShuffleboardTab m_shuffleboardTab;
    private final static String SHUFFLEBOARD_TAB_NAME = "Competition";

    /** Creates a new Dashboard. */
    public Dashboard() {
      m_shuffleboardTab = Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
      Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
  }
}