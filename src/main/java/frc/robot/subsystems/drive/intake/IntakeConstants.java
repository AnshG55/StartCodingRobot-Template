// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class IntakeConstants {
  private IntakeConstants() {}

  public static final int MOTOR_ID = 0;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final MotorType MOTOR = MotorType.KrakenX44;

  public static final double ROLLER_MOI = 0.5 * 0.3 * Math.pow(Units.inchesToMeters(1.0), 2);

  public static final double GEAR_RATIO = 1.0;

  // In M/s
  public static final double INTAKE_SPEED_Mps = 20;
}
