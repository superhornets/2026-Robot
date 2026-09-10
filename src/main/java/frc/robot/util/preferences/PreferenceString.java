// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;
import java.util.function.Supplier;

/**
 * A dashboard-tunable string backed by WPILib Preferences (NT path {@code /Preferences/<key>}).
 *
 * <p>Values changed on the dashboard (e.g. via Elastic's Preferences widget) are picked up on the
 * next call to {@link #get()} without a robot restart.
 */
public class PreferenceString implements Supplier<String> {
  private final String key;
  private final String defaultValue;

  public PreferenceString(String key, String defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    Preferences.initString(key, defaultValue);
    PreferenceRegistry.registerReset(() -> set(defaultValue));
  }

  /** Returns the current value from Preferences. */
  public String get() {
    return Preferences.getString(key, defaultValue);
  }

  /** Programmatically sets the value, persisting it to Preferences. */
  public void set(String value) {
    Preferences.setString(key, value);
  }
}
