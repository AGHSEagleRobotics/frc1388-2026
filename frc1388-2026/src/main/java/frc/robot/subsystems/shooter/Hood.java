// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.shooter.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private final HoodIO m_io;
  private final HoodIOInputs inputs = new HoodIOInputs();
  public HoodState hoodState;
  public double m_distanceFromHub;
  public double m_distanceFromHubSOTM;
  public double m_distanceFromPass;

  // private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
  // private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
  // private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

  // private final SysIdRoutine sysIdRoutine;  
  
  public enum HoodState {
    IDLE,
    SHOOTING,
    SOTM,
    PASSING,
    MANUAL_CLOSE,
    MANUAL_FAR,
    TESTING
  }


  public Hood(HoodIO io) {
    m_io = io;
    hoodState = HoodState.IDLE;

    // sysIdRoutine = new SysIdRoutine(
    //     new SysIdRoutine.Config(Volts.of(0.15).per(Second), Volts.of(0.35), Seconds.of(3.5), null),
    //     new SysIdRoutine.Mechanism(
    //         (Voltage volts) -> {
    //           m_io.setVoltage(volts.in(Volts));
    //         },
    //         log -> {
    //           log.motor("hood-pivot")
    //               .voltage(sysidAppliedVoltageMeasure.mut_replace(inputs.hoodMotorVoltage,
    //                   Volts))
    //               .angularPosition(
    //                   sysidPositionMeasure.mut_replace(inputs.hoodMotorPosition,
    //                       Rotations))
    //               .angularVelocity(sysidVelocityMeasure.mut_replace(inputs.hoodMotorVelocityRPS,
    //                   RotationsPerSecond));
    //         },
    //         this));
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
    else if (hoodState == HoodState.PASSING) {
      setShootingPosition(HoodConstants.HOOD_FAR);
    }
    else if (hoodState == HoodState.TESTING) {
      m_io.setVoltage(-0.22);
    }
    else if (hoodState == HoodState.MANUAL_CLOSE) {
      setShootingPosition(HoodConstants.HOOD_CLOSE);
    }
    else if (hoodState == HoodState.MANUAL_FAR) {
      setShootingPosition(HoodConstants.HOOD_FAR);
    }

    DogLog.log("Hood/Absolute Encoder", m_io.getPosition());
  }

  public void stop() {
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

  public void setHoodState(HoodState hoodState) {
    this.hoodState = hoodState;
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

  public HoodState getHoodState() {
    return hoodState;
  }

  // public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
  //       return sysIdRoutine.quasistatic(direction).withName("shooterHood.sysIdQuasistatic");
  //   }

  //   public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
  //       return sysIdRoutine.dynamic(direction).withName("shooterHood.sysIdDynamic");
  //   }
}
