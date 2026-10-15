// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final ArmIO io;

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmIO.Gains appliedGains = ArmConstants.GAINS;

  private final TunableNumber kV = new TunableNumber("Arm/kV", ArmConstants.GAINS.kV());
  private final TunableNumber kA = new TunableNumber("Arm/kA", ArmConstants.GAINS.kA());
  private final TunableNumber kG = new TunableNumber("Arm/kG", ArmConstants.GAINS.kG());
  private final TunableNumber kS = new TunableNumber("Arm/kS", ArmConstants.GAINS.kS());
  private final TunableNumber kD = new TunableNumber("Arm/kD", ArmConstants.GAINS.kD());
  private final TunableNumber kP = new TunableNumber("Arm/kP", ArmConstants.GAINS.kP());
  private final TunableNumber targetVelocity =
      new TunableNumber("Arm/TargetVelocity", ArmConstants.GAINS.targetVelocity());
  private final TunableNumber targetAcceleration =
      new TunableNumber("Arm/TargetAcceleration", ArmConstants.GAINS.targetAcceleration());

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    var gains =
        new ArmIO.Gains(kV.get(), kA.get(), kG.get(), kS.get(), kP.get(), kD.get(), targetVelocity.get(), targetAcceleration.get());
    if (!gains.equals(appliedGains)) {
      io.setGains(gains);
      appliedGains = gains;
    }
  }
}
