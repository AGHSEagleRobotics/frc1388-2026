// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.Map;
import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Dashboard {
    // Everything goes on THIS tab
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Jumbotron");
    
    private GenericEntry dashboardTest;

    private final GenericEntry m_mainTimerEntry;
    private final GenericEntry m_shiftTimerEntry;
    private final GenericEntry m_stageEntry;
    private final GenericEntry m_alertEntry; // For the flashing background

    private final Timer m_matchTimer = new Timer();
    private final Timer m_shiftTimer = new Timer();
    
    private int m_currentShift = 1;
    private boolean m_isTeleop = false;

    public Dashboard() {
        // 1. BIG MAIN TIMER (Positioned at Top)
        m_mainTimerEntry = m_tab.add("MATCH CLOCK", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(4, 2).getEntry();

        // 2. SHIFT TIMER (Below Main)
        m_shiftTimerEntry = m_tab.add("SHIFT TIME", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withPosition(0, 2)
            .withSize(2, 2).getEntry();

        // 3. STAGE COUNTER (To the side of Shift)
        m_stageEntry = m_tab.add("STAGE", "1 / 6")
            .withPosition(2, 2)
            .withSize(2, 2).getEntry();

        // 4. FLASHING ALERT (A box that changes color)
        m_alertEntry = m_tab.add("LOW TIME ALERT", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0)
            .withSize(1, 4) // Vertical strip on the side
            .withProperties(Map.of("Color when true", "Red", "Color when false", "Black"))
            .getEntry();
    }

    public void update() {
        double matchTime = m_matchTimer.get();
        double displayTime;
        String label = "MATCH CLOCK";

        // --- PHASE LOGIC ---
        if (matchTime <= 15) { 
            // AUTO PHASE (15s)
            displayTime = 15 - matchTime;
            label = "AUTO: " + String.format("%.1f", displayTime);
        } else if (matchTime <= 18) { 
            // 3s TRANSITION PAUSE
            displayTime = 0;
            label = "GET READY...";
            if (!m_isTeleop) { m_isTeleop = true; m_shiftTimer.start(); }
        } else { 
            // TELEOP PHASE (Starts fresh after pause)
            displayTime = 135 - (matchTime - 18);
            label = "TELEOP: " + (int)displayTime;
        }

        // --- SHIFT & FLASHING LOGIC ---
        double shiftLeft = 20.0 - m_shiftTimer.get();
        
        // Flashing effect: If time < 5s, toggle every 0.25 seconds
        boolean shouldFlash = (shiftLeft < 5.0) && ((System.currentTimeMillis() / 250) % 2 == 0);

        if (m_isTeleop && shiftLeft <= 0 && m_currentShift < 6) {
            m_shiftTimer.reset();
            m_currentShift++;
        }

        // --- PUSH TO DASHBOARD ---
        m_mainTimerEntry.setString(label); 
        m_shiftTimerEntry.setDouble(Math.max(0, shiftLeft));
        m_stageEntry.setString(m_currentShift + " / 6");
        m_alertEntry.setBoolean(shouldFlash);
    }

    public void start() {
        m_matchTimer.reset();
        m_matchTimer.start();
    }
  }



// public class Dashboard extends SubsystemBase {
//   /** Creates a new Dashboard. */
    
//   private final ShuffleboardTab m_shuffleboardTab;
//     private final static String SHUFFLEBOARD_TAB_NAME = "Competition";

//     /** Creates a new Dashboard. */
//     public Dashboard() {
//       m_shuffleboardTab = Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
//       Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

//       m_shuffleboardTab.add("FMS Info",
//       NetworkTableInstance.getDefault().getTable("FMSInfo"))
//         .withWidget("FMSInfo")
//         .withPosition(1, 1)
//         .withSize(4,3);


//   }

// //   public boolean isHubActive() {
// //   Optional<Alliance> alliance = DriverStation.getAlliance();
// //   // If we have no alliance, we cannot be enabled, therefore no hub.
// //   if (alliance.isEmpty()) {
// //     return false;
// //   }
// //   // Hub is always enabled in autonomous.
// //   if (DriverStation.isAutonomousEnabled()) {
// //     return true;
// //   }
// //   // At this point, if we're not teleop enabled, there is no hub.
// //   if (!DriverStation.isTeleopEnabled()) {
// //     return false;
// //   }

// //   // We're teleop enabled, compute.
// //   double matchTime = DriverStation.getMatchTime();
// //   String gameData = DriverStation.getGameSpecificMessage();
// //   // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
// //   if (gameData.isEmpty()) {
// //     return true;
// //   }
// //   boolean redInactiveFirst = false;
// //   switch (gameData.charAt(0)) {
// //     case 'R' -> redInactiveFirst = true;
// //     case 'B' -> redInactiveFirst = false;
// //     default -> {
// //       // If we have invalid game data, assume hub is active.
// //       return true;
// //     }
// //   }

// //   // Shift was is active for blue if red won auto, or red if blue won auto.
// //   boolean shift1Active = switch (alliance.get()) {
// //     case Red -> !redInactiveFirst;
// //     case Blue -> redInactiveFirst;
// //   };

// //   if (matchTime > 130) {
// //     // Transition shift, hub is active.
// //     return true;
// //   } else if (matchTime > 105) {
// //     // Shift 1
// //     return shift1Active;
// //   } else if (matchTime > 80) {
// //     // Shift 2
// //     return !shift1Active;
// //   } else if (matchTime > 55) {
// //     // Shift 3
// //     return shift1Active;
// //   } else if (matchTime > 30) {
// //     // Shift 4
// //     return !shift1Active;
// //   } else {
// //     // End game, hub always active.
// //     return true;
// //   }
// // }


// //   @Override
// //   public void periodic() {
// //     // This method will be called once per scheduler run
// // System.out.println("Dashboard working?");
// // dashboardTest.setString("Testing");
// //   }

  
// }
  
