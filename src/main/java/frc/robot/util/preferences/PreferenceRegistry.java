// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.preferences;

import java.util.ArrayList;
import java.util.List;

/**
 * Tracks all Preference* instances so they can be reset to their defaults in one call.
 *
 * <p>Each PreferenceNumber, PreferenceBoolean, and PreferenceString registers a reset callback at
 * construction time via {@link #registerReset}. Calling {@link #resetAll} invokes every registered
 * callback, pushing each instance's default value back to Preferences.
 */
public final class PreferenceRegistry {
  private static final List<Runnable> resetCallbacks = new ArrayList<>();

  private PreferenceRegistry() {}

  /** Called by Preference* constructors to register a callback that restores the default value. */
  static void registerReset(Runnable resetToDefault) {
    resetCallbacks.add(resetToDefault);
  }

  /** Resets every registered Preference* instance back to its default value. */
  public static void resetAll() {
    resetCallbacks.forEach(Runnable::run);
  }
}
