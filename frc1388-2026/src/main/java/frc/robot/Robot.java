// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;



public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
   public void robotInit() {
     DataLogManager.start();
     DataLogManager.log("####### RobotInit");
     DataLogManager.log("Git version: " + BuildInfo.GIT_VERSION + " (branch: " + BuildInfo.GIT_BRANCH + " "
         + BuildInfo.GIT_STATUS + ")");
     DataLogManager.log("      Built: " + BuildInfo.BUILD_DATE + "  " + BuildInfo.BUILD_TIME);

     // logs everytime a command starts / stops
     CommandScheduler.getInstance()
         .onCommandInitialize(command -> DataLogManager.log("++ " + command.getName() + " Initialized"));
     CommandScheduler.getInstance()
         .onCommandInterrupt(command -> DataLogManager.log("-- " + command.getName() + " Interrupted"));
     CommandScheduler.getInstance()
         .onCommandFinish(command -> DataLogManager.log("-- " + command.getName() + " Finished"));
   }
 

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        // m_robotContainer.m_ledSubsystem.turnOffSolidWhite();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        DataLogManager.log("####### Autonomous Init");
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

         if (DriverStation.isFMSAttached()) {
      String fmsInfo = "FMS info: ";
      fmsInfo += " " + DriverStation.getEventName();
      fmsInfo += " " + DriverStation.getMatchType();
      fmsInfo += " match " + DriverStation.getMatchNumber();
      fmsInfo += " replay " + DriverStation.getReplayNumber();
      fmsInfo += ";  " + DriverStation.getAlliance() + " alliance";
      fmsInfo += ",  Driver Station " + DriverStation.getLocation();
      DataLogManager.log(fmsInfo);
    } else {
      DataLogManager.log("FMS not connected");

      DataLogManager.log("Match type:\t" + DriverStation.getMatchType());
      DataLogManager.log("Event name:\t" + DriverStation.getEventName());
      DataLogManager.log("Alliance:\t" + DriverStation.getAlliance());
      DataLogManager.log("Match number:\t" + DriverStation.getMatchNumber());
    }

            m_robotContainer.m_ledSubsystem.setSolidWhite();

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
         m_robotContainer.m_ledSubsystem.setSolidWhite();
        //  m_robotContainer.m_ledSubsystem.setSolidBlueTeam();
        //  m_robotContainer.m_ledSubsystem.setSolidRedTeam();
         m_robotContainer.m_ledSubsystem.setEagleColors();
        //  m_robotContainer.m_ledSubsystem.setSolidNoTeam();

    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
