// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
// Requires simulation support:
        // for(int i = 0; i < 50; i++) {
        //  SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(2 + (i*(0.01)),2)));
        // }
    }

    @Override
    public void robotInit() {
        logInit();

        DataLogManager.log("####### RobotInit");
        DataLogManager.log("Git version: " + BuildInfo.GIT_VERSION
                + " (branch: " + BuildInfo.GIT_BRANCH + " "
                + BuildInfo.GIT_STATUS + ")");
        DataLogManager.log("      Built: " + BuildInfo.BUILD_DATE + "  " + BuildInfo.BUILD_TIME);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        DataLogManager.log("####### Robot Disabled");
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        DataLogManager.log("####### Autonomous Init");
        logMatchInfo();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        DataLogManager.log("####### Teleop Init");
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        DataLogManager.log("####### Test Init");
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        DogLog.log("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        
    }

    private void logInit() {
        DogLog.setOptions(new DogLogOptions()
            .withLogExtras(true)
            .withCaptureDs(true)
            .withNtPublish(true)
            .withCaptureNt(true));
// Causes errors if PDH is not present:
//        DogLog.setPdh(new PowerDistribution());

        // logs everytime a command starts / stops
        CommandScheduler.getInstance()
                .onCommandInitialize(command -> DataLogManager.log("++ " + command.getName() + " Initialized"));
        CommandScheduler.getInstance()
                .onCommandInterrupt(command -> DataLogManager.log("-- " + command.getName() + " Interrupted"));
        CommandScheduler.getInstance()
                .onCommandFinish(command -> DataLogManager.log("-- " + command.getName() + " Finished"));
    }

    private void logMatchInfo() {
        // Get match info from FMS
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
    }

    public static Alliance getAllianceColor() {
    if (DriverStation.getAlliance().isPresent()){
      return DriverStation.getAlliance().get();
    } else {
      return Alliance.Blue;
    }
  }
}
