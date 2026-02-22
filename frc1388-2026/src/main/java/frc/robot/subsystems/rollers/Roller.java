// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.rollers.RollerIO.RollerIOInputs;
import frc.robot.subsystems.rollers.RollerIO.RollerIOMode;
import frc.robot.subsystems.rollers.RollerIO.RollerIOOutputs;

public class Roller extends SubsystemBase {
  /** Creates a new RollerSubsystem. */
  public class RollerSystem {

  private double kS = 0.0;
  private double kV = 0.0;

  private final RollerIOInputs inputs = new RollerIOInputs();
  private final RollerIOOutputs outputs = new RollerIOOutputs();

  private BooleanSupplier coastOverride = () -> false;

  public RollerSystem(double kP, double kD) {

    outputs.kP = kP;
    outputs.kD = kD;
  }

  public void periodic() {
    // Update mode
    if (DriverStation.isDisabled()) {
      outputs.mode = RollerIOMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = RollerIOMode.COAST;
      }
    }
  }

  public void runOpenLoop(double volts) {
    outputs.mode = RollerIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = volts;
  }

  public void runClosedLoop(double setpointVelocity) {
    outputs.mode = RollerIOMode.CLOSED_LOOP;
    outputs.velocity = setpointVelocity;
    outputs.feedforward = Math.signum(setpointVelocity) * kS + setpointVelocity * kV;
  }

  public void setGains(double kP, double kD) {
    outputs.kP = kP;
    outputs.kD = kD;
  }

  public void setFeedforward(double kS, double kV) {
    this.kS = kS;
    this.kV = kV;
  }

  public double getTorqueCurrent() {
    return inputs.bottomrollerTorqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.bottomrollerVelocityRPS;
  }

  public void stop() {
    outputs.mode = RollerIOMode.BRAKE;
  }
}
}
