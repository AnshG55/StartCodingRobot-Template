package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double positionRad = 0.0;

    public double velocityRadPerSec = 0.0;

    public double appliedVolts = 0.0;

    public double currentAmps = 0.0;

    public double targetPosRad = 0.0;

    public double targetVelRadPerSec = 0.0;
  }

  record Gains(
      double kV,
      double kA,
      double kG,
      double kS,
      double kP,
      double kD,
      double targetVelocity,
      double targetAcceleration) {}

  default void updateInputs(ArmIOInputs inputs) {}

  default void setGains(Gains gains) {}

  default void setPosition(double angleRad) {}

  default void setVoltage(double volts) {}
}
