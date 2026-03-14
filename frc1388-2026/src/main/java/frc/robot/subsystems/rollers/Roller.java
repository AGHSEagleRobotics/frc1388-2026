// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.rollers.RollerIO.RollerIOInputs;

public class Roller extends SubsystemBase {
  /** Creates a new RollerSubsystem. */

  private final RollerIO m_io;

  public RollerState rollerState;

  private final RollerIOInputs inputs = new RollerIOInputs();

  enum RollerMotor {
    TOPROLLER,
    BOTTOMROLLER
  }

  public enum RollerState {
    IDLE,
    INTAKING,
    SHOOTING,
    TESTING
  }

  public Roller(RollerIO io) {
    m_io = io;
    rollerState = RollerState.IDLE;
  }

  public void periodic() {
    m_io.updateInputs(inputs);
    if (rollerState == RollerState.IDLE) {
      stop();
    } else if (rollerState == RollerState.INTAKING) {
      setIntakingRollers();
    } else if ((rollerState == RollerState.SHOOTING)) {
      setShootingRollers();
    } else if (rollerState == RollerState.TESTING) {
      setTestingRollers();
    }
  }

    public void stop() {
      m_io.setBottomRollerVoltage(0);
      m_io.setTopRollerVoltage(0);
    }

    public void setIntakingRollers() {
      m_io.setBottomRollerVoltage(RollerConstants.bottomRollerIntakeSpeed);
      m_io.setTopRollerVoltage(0);
    }

    public void setShootingRollers() {
      m_io.setBottomRollerVoltage(RollerConstants.bottomRollerShootingSpeed);
      m_io.setTopRollerVoltage(RollerConstants.topRollerShootingSpeed);
    }

    public void setTestingRollers() {
      m_io.setBottomRollerVoltage(RollerConstants.bottomRollerShootingSpeed);
      m_io.setTopRollerVoltage(RollerConstants.topRollerShootingSpeed);
    }

    public RollerState getRollerState() {
      return rollerState;
    }

    public void setRollerState(RollerState rollerState) {
      this.rollerState = rollerState;
    }
}