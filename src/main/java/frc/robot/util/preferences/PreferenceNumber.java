// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;

import java.util.function.DoubleSupplier;

/**
 * A dashboard-tunable number backed by WPILib Preferences (NT path {@code /Preferences/<key>}).
 *
 * <p>Values changed on the dashboard (e.g. via Elastic's Preferences widget) are picked up on the
 * next call to {@link #get()} without a robot restart.
 */
public class PreferenceNumber implements DoubleSupplier {
  private final String key;
  private final Double defaultValue;

  public PreferenceNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;

    Preferences.initDouble(key, defaultValue);
    PreferenceRegistry.registerReset(() -> set(defaultValue));
  }

  /** Returns the current value from Preferences. */
  public double get() {
    return Preferences.getDouble(key, defaultValue);
  }

  /** Programmatically sets the value, persisting it to Preferences. */
  public void set(double value) {
    Preferences.setDouble(key, value);
  }

  @Override
  public double getAsDouble() {
      return get();
  }
}
