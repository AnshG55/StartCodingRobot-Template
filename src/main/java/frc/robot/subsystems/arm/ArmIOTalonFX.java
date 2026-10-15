// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOTalonFX implements ArmIO {
  /** Creates a new ArmIOTalonFX. */
  private final StatusSignal<Angle> position;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Double> targetPos;
  private final StatusSignal<Double> targetVel;

  protected final TalonFX motor = new TalonFX(ArmConstants.CAN_ID);

  private final MotionMagicVoltage positionRequest;
  private final VoltageOut voltageRequest;
  private final TalonFXConfiguration config;
  private final Slot0Configs slot0Configs;

  public ArmIOTalonFX() {
    position = motor.getPosition(false);
    velocity = motor.getVelocity(false);
    appliedVolts = motor.getMotorVoltage(false);
    current = motor.getStatorCurrent(false);
    targetPos = motor.getClosedLoopOutput();
    targetVel = motor.getClosedLoopDerivativeOutput();

    slot0Configs =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(ArmConstants.MIN_ANGLE_RAD);
    slot0Configs
        .withKV(ArmConstants.GAINS.kV())
        .withKA(ArmConstants.GAINS.kA())
        .withKG(ArmConstants.GAINS.kG())
        .withKS(ArmConstants.GAINS.kS())
        .withKP(ArmConstants.GAINS.kP())
        .withKD(ArmConstants.GAINS.kD());

    config = new TalonFXConfiguration();
    config.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(ArmConstants.MOTOR_DIRECTION);
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(ArmConstants.GAINS.targetAcceleration())
            .withMotionMagicCruiseVelocity(ArmConstants.GAINS.targetVelocity());
    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.GEAR_RATIO);

    config.Slot0 = slot0Configs;

    motor.getConfigurator().apply(config);

    positionRequest = new MotionMagicVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);

    motor.setPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.targetPosRad = targetPos.getValueAsDouble();
    inputs.targetVelRadPerSec = targetVel.getValueAsDouble();
  }

  @Override
  public void setGains(Gains gains) {
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(gains.targetAcceleration())
            .withMotionMagicCruiseVelocity(gains.targetVelocity());

    slot0Configs
        .withKV(gains.kV())
        .withKA(gains.kA())
        .withKG(gains.kG())
        .withKS(gains.kS())
        .withKP(gains.kP())
        .withKD(gains.kD());

    config.Slot0 = slot0Configs;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(double angleRad) {
    motor.setControl(positionRequest.withPosition(Units.radiansToRotations(angleRad)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
