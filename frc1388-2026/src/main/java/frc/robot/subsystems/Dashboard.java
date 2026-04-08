// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.superstructure.Superstructure;

public class Dashboard extends SubsystemBase {
  private final ShuffleboardTab m_shuffleboardTab;
    private final static String SHUFFLEBOARD_TAB_NAME = "Competition";
    private final GenericEntry isHubActive;
    private final GenericEntry hubEnabledTimer;
    private final GenericEntry shooterState;
    private final GenericEntry intakeState;
    private final Shooter m_shooter;
    private final Intake m_intake;


    // private final static String shuffleboardAutonomous = "Autonomous";
    
    // private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    
    /** Creates a new Dashboard. */
    public Dashboard(Shooter shooter, Intake intake) {

m_shooter = shooter;
m_intake = intake;

      Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);
      m_shuffleboardTab = Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
      ShuffleboardTab AutoTab = Shuffleboard.getTab("Autonomous");


       hubEnabledTimer = AutoTab
          .add("ShiftCountdown", "Default")
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 0)
          .getEntry();

       isHubActive = AutoTab
          .add("Hub Enabled?", false)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withPosition(4, 0)
          .getEntry();

       shooterState = AutoTab
          .add("Shooter State", "Default")
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(4, 0)
          .getEntry();

       intakeState = AutoTab
          .add("Intake State", "Default")
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(4, 0)
          .getEntry();



   
   

        // shuffleboardAutonomous.add("Autonomous")
        //  .withWidget(BuiltInWidgets.kTextView)
        //   .withSize(8, 4)
        //   .withPosition(0, 0);
          

        
 
  }

  public boolean isHubActive() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130) {
    // Transition shift, hub is active.
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    return !shift1Active;
  } else {
    // End game, hub always active.
    return true;
  }
}

public int timeLeftToShoot() {
  Optional<Alliance> alliance = DriverStation.getAlliance();

   if (alliance.isEmpty()) {
    return 0;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return 0;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return 0;
  }

  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();

  if (gameData.isEmpty()) {
    return 0;
  }

  if ((alliance.get() == Alliance.Red) && (gameData.charAt(0) == 'R')
      || (alliance.get() == Alliance.Blue) && (gameData.charAt(0) == 'B')) {
    //timer for if we won auto
    if (matchTime >= 130 && matchTime <= 140) { 
      return (int) matchTime - 130;
    }
    if (matchTime >= 105 && matchTime <= 130) {
      return (int) matchTime - 105;
    }
    if (matchTime >= 80 && matchTime <= 105) {
      return (int) matchTime - 80;
    }
    if (matchTime <= 80 && matchTime >= 55) {
      return (int) matchTime - 55;
    }
    if (matchTime <= 55) {
      return (int) matchTime;
    }
    //timer for if we lost uto
  } else { 
    if (matchTime >= 105 && matchTime <= 140) {
      return (int) matchTime - 105;
    }
    if (matchTime >= 80 && matchTime <= 105) {
      return (int) matchTime - 80;
    }
    if (matchTime >= 55 && matchTime <= 80) {
      return (int) matchTime - 55;
    }
    if (matchTime >= 30 && matchTime <= 55) {
      return (int)matchTime - 30;
    }
    if (matchTime < 30) {
      return (int)matchTime;
    }
  }
  return -1;
}

@Override
  public void periodic() {
hubEnabledTimer.setString(String.valueOf(timeLeftToShoot()));
isHubActive.setBoolean(isHubActive());
SmartDashboard.putNumber("Match Timer", DriverStation.getMatchTime());
shooterState.setString(String.valueOf(m_shooter.getShooterState().name()));
intakeState.setString(String.valueOf(m_intake.getIntakeState().name()));
}
}

