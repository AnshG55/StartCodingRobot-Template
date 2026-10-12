package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConstants {
    private ArmConstants() {}

    public static final int CAN_ID = 0;

    public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

    public static final double GEAR_RATIO = 1;

    //IN M/S || IN M/S^2
    public static final double targetVelocity = 1;
    public static final double targetAcceleration = 1;

    public static final double LENGTH_METERS = 0.5;
    public static final double MASS_KG = 4.0;
    public static final double MOI = SingleJointedArmSim.estimateMOI(LENGTH_METERS, MASS_KG);

    public static final double MIN_ANGLE_RAD = Units.degreesToRadians(0);
    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(90);

    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kP = 0;
    public static final double kD = 0;

    public static final ArmIO.Gains GAINS =
      new ArmIO.Gains(kV, kA, kG, kS, kP, kD, targetVelocity, targetAcceleration);
}
