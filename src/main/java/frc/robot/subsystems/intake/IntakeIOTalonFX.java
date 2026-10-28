// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {

    protected final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);
    
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    public IntakeIOTalonFX() {
        velocity = intakeMotor.getVelocity();
        appliedVolts = intakeMotor.getMotorVoltage();
        current = intakeMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.apppliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }
}
