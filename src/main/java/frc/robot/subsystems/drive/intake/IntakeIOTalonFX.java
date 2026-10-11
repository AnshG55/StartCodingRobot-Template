// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  /** Creates a new IntakeIOTalonFX. */
  protected final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);

  private final MotionMagicVoltage voltRequestMM;
  private final TalonFXConfiguration motorConfig;
  private final VoltageOut voltageRequest;

  private final Slot0Configs config;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Double> profilePos;
  private final StatusSignal<Angle> realPos;

  public IntakeIOTalonFX() {
    config = new Slot0Configs();
    config.kV = IntakeConstants.GAINS.kV();
    config.kA = IntakeConstants.GAINS.kA();
    config.kP = IntakeConstants.GAINS.kP();
    config.kI = IntakeConstants.GAINS.kI();
    config.kD = IntakeConstants.GAINS.kD();

    motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(IntakeConstants.MOTOR_DIRECTION);
    motorConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                Units.radiansToRotations(
                    IntakeConstants.GAINS.accelerationmps() / IntakeConstants.WHEEL_RADIUS));
    motorConfig.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.GEAR_RATIO);
    motorConfig.Slot0 = config;

    velocity = intakeMotor.getVelocity(false);
    appliedVolts = intakeMotor.getMotorVoltage(false);
    current = intakeMotor.getStatorCurrent(false);
    profilePos = intakeMotor.getClosedLoopReference();
    realPos = intakeMotor.getPosition();

    intakeMotor.getConfigurator().apply(motorConfig);

    intakeMotor.setPosition(0);

    voltRequestMM = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocitymps =
        Units.rotationsToRadians(velocity.getValueAsDouble()) * IntakeConstants.WHEEL_RADIUS;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.profilePos = profilePos.getValueAsDouble();
    inputs.realPos = realPos.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double pos) {
    intakeMotor.setControl(voltRequestMM.withPosition(Units.rotationsToDegrees(pos)));
  }

  @Override
  public void setGains(Gains gains) {
    config.kV = IntakeConstants.GAINS.kV();
    config.kA = IntakeConstants.GAINS.kA();
    config.kP = IntakeConstants.GAINS.kP();
    config.kI = IntakeConstants.GAINS.kI();
    config.kD = IntakeConstants.GAINS.kD();
    motorConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                Units.radiansToRotations(
                    IntakeConstants.GAINS.accelerationmps() / IntakeConstants.WHEEL_RADIUS));
    motorConfig.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.GEAR_RATIO);
    motorConfig.Slot0 = config;
    intakeMotor.getConfigurator().apply(motorConfig);
  }
}
