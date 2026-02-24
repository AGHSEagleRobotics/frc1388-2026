// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

/** Add your docs here. */
public interface RollerIO {

     public static class RollerIOInputs {
    double bottomrollerTorqueCurrentAmps = 0;
    double toprollerTorqueCurrentAmps = 0;

    double bottomrollerSupplyCurrentAmps = 0;
    double toprollerSupplyCurrentAmps = 0;

    double bottomrollerVelocityRPS = 0;
    double toprollerVelocityRPS = 0;

    double bottomrollerReferenceVelocityRPS = 0;
    double toprollerReferenceVelocityRPS = 0;

    double bottomrollerTempCelsius = 0;
    double toprollerTempCelsius = 0;

    double bottomrollerVoltage = 0;
    double toprollerVoltage = 0;
  }

  public enum RollerIOMode {
    BRAKE,
    COAST,
    VOLTAGE_CONTROL,
    CLOSED_LOOP
  }

  public static class RollerIOOutputs {
    public RollerIOMode mode = RollerIOMode.BRAKE;
    // Voltage control
    public double appliedVoltage = 0.0;

    // Closed loop control
    public double velocity = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double feedforward = 0.0;

    public boolean brakeModeEnabled = true;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void applyOutputs(RollerIOInputs outputs) {}

  public default void setBottomRollerVoltage(double rps){}

  public default void setTopRollerVoltage(double rps){}
}
