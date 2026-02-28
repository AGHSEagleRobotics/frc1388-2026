// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private final HoodIO m_io;
  private final HoodIOInputs inputs = new HoodIOInputs();
  public HoodState hoodState;
  public double m_distanceFromHub;
  public double m_distanceFromHubSOTM;
  public double m_distanceFromPass;

  public enum HoodState {
    IDLE,
    SHOOTING,
    SOTM,
    TESTING
  }


  public Hood(HoodIO io) {
    m_io = io;
    hoodState = HoodState.IDLE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_io.updateInputs(inputs);
    if (hoodState == HoodState.IDLE) {
      stop();
    }
    else if (hoodState == HoodState.SHOOTING) {
      setShootingPosition(HoodConstants.DISTANCE_TO_SHOT_HOODANGLE.get(m_distanceFromHub));
    }
    else if (hoodState == HoodState.SOTM) {
      setShootingPosition(HoodConstants.DISTANCE_TO_SHOT_HOODANGLE.get(m_distanceFromHubSOTM));
    }
    // add passing here
    else if (hoodState == HoodState.TESTING) {

    }

  }

  public void stop() {
    m_io.setVoltage(0);
    m_io.setPosition(m_io.getPosition());
  }

  public void setShootingPosition(double position) {
    m_io.setPosition(position);
  }

  public void setSOTMPosition(double position) {
    m_io.setPosition(position);
  }

  public void setTestingPosition(double position) {
    m_io.setPosition(position);
  }
  public void setDistanceFromHub(double distanceFromHub) {
    m_distanceFromHub = distanceFromHub;
  }

  public void setDistanceFromHubSOTM(double distanceFromHubSOTM) {
    m_distanceFromHubSOTM = distanceFromHubSOTM;
  }

  public void setDistanceFromPass(double distanceFromPass) {
    m_distanceFromPass = distanceFromPass;
  }
}
