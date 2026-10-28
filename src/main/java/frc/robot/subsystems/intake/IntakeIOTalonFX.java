// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
protected final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);

private final VoltageOut voltageRequest = new VoltageOut(Output:0.0);

private final StatusSignal<AngularVelocity> velocity;
private final StatusSignal<Voltage> appliedVolts;
private final StatusSignal<Current> current;

    public IntakeIOTalonFX() {
        velocity = intakeMOtor.getVelocity();
        appliedVolts = intakeMotor.getMotorVoltage();
        current = intakeMotor.getStatorCurrent();
    }
@Override
public void updateInputs(IntakeIOInputs) {
    inputs.velocityRadPerSec = Units.rotationToRadians(velocity.getValueAsDouble());\
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
}

@Overridepublic void setVOltage(double volts) {
    intakeMotor.setControl(voltageReqest.withOutput(volts));
}
}
