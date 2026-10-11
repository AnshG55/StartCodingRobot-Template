// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final IntakeIO io;

  private final TunableNumber kV = new TunableNumber("Intake/kV", IntakeConstants.GAINS.kP());
  private final TunableNumber kA = new TunableNumber("Intake/kA", IntakeConstants.GAINS.kA());
  private final TunableNumber kP = new TunableNumber("Intake/kP", IntakeConstants.GAINS.kP());
  private final TunableNumber kI = new TunableNumber("Intake/kI", IntakeConstants.GAINS.kI());
  private final TunableNumber kD = new TunableNumber("Intake/kD", IntakeConstants.GAINS.kD());
  private final TunableNumber targetVelocitymps =
      new TunableNumber("Intake/targetVelocitymps", IntakeConstants.GAINS.targetVelocitymps());
  private final TunableNumber accelerationmps =
      new TunableNumber("Intake/accelerationmps", IntakeConstants.GAINS.accelerationmps());

  private IntakeIO.Gains appliedGains = IntakeConstants.GAINS;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    var gains =
        new IntakeIO.Gains(
            kV.get(),
            kA.get(),
            kP.get(),
            kI.get(),
            kD.get(),
            targetVelocitymps.get(),
            accelerationmps.get());
    if (!gains.equals(appliedGains)) {
      io.setGains(gains);
      appliedGains = gains;
      System.out.println("Set Gains");
    }
  }

  public Command intakeVoltage() {
    return runEnd(
        () -> {
          io.setVoltage(2);
        },
        () -> {
          io.setVoltage(0.0);
        });
  }

  public Command setIntakePos(double pos) {
    return runEnd(
        () -> {
          io.setPosition(pos);
        },
        () -> {
          io.setVoltage(0.0);
        });
  }
}
