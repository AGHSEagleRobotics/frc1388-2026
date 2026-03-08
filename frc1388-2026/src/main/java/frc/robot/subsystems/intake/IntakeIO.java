// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface IntakeIO {
    public static class IntakeIOInputs {
        double deployMotorTorqueCurrentAmps = 0;
        double rollerMotorTorqueCurrentAmps = 0;

        double deployMotorSupplyCurrentAmps = 0;
        double rollerMotorSupplyCurrentAmps = 0;

        double deployMotorVelocityRPS = 0;
        double rollerMotorVelocityRPS = 0;

        double deployMotorReferenceVelocityRPS = 0;
        double rollerMotorReferenceVelocityRPS = 0;

        double bottomrollerTempCelsius = 0;
        double rollerMotorTempCelsius = 0;

        double deployMotorVoltage = 0;
        double rollerMotorVoltage = 0;

        double deployMotorPosition;

    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setDeployPosition(double position);

    public void setDeployVoltage(double out);

    public void setRollerVoltage(double out);

    public void stopRack();

    public void stopSpin();

    public void zeroPosition();

    public double getPosition();
}
