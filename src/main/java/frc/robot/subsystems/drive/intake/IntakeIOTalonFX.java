// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.intake;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO{

    protected final TalonFX intakeMotor = new TalonFx(IntakeConstants.MOTOR_ID);

    private final VoltageOut voltageRequest = new VoltageOut(Output:0.0);
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    public IntakeIOTalonFX() {
        velocity = intakeMotor.getVelocity();
        appliedVolts = intakeMotor.getMotorVoltage();
        current = intakeMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(frequencyHz:50.0, velocity, appliedVolts, current);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps - current.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }
}
