package frc.robot.util;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableNumber implements DoubleSupplier {
  private final double defaultValue;

  private final LoggedNetworkNumber dashboardNumber;

  public TunableNumber(String key, double defaultValue) {
    this.defaultValue = defaultValue;
    dashboardNumber = new LoggedNetworkNumber("/Tuning/" + key, defaultValue);
  }

  public double get() {
    return dashboardNumber != null ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
