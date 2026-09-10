// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class PreferencesManager extends SubsystemBase {
    private static final double CONFIRMATION_TIMEOUT_SECONDS = 3.0;

    private LoggedNetworkBoolean reset =
        new LoggedNetworkBoolean("PreferencesManager/Reset", false);
    private LoggedNetworkBoolean confirmation =
        new LoggedNetworkBoolean("PreferencesManager/Confirm", false);
    private PreferenceString resetMessage =
        new PreferenceString("PreferencesManager/Message", "");

    private final Timer resetTimer = new Timer();

    @Override
    public void periodic() {
        if (reset.getAsBoolean() && !resetTimer.isRunning()) {
            // Reset was just toggled on — start the confirmation window.
            confirmation.set(false);
            resetTimer.restart();
        } else if (isAwaitingConfirmation() && confirmation.getAsBoolean()) {
            // Operator confirmed the reset.
            resetAll();
            reset.set(false);
            confirmation.set(false);
            resetTimer.stop();
        } else if (confirmation.getAsBoolean()) {
            // Operator confirmed the reset outside of the confirmation window.
            reset.set(false);
            confirmation.set(false);
            resetTimer.stop();
        } else if (isAwaitingConfirmation() && resetTimer.hasElapsed(CONFIRMATION_TIMEOUT_SECONDS)) {
            // Confirmation window expired without confirmation.
            reset.set(false);
            confirmation.set(false);
            resetTimer.stop();
        }

        super.periodic();
    }

    private boolean isAwaitingConfirmation() {
        return reset.getAsBoolean() && resetTimer.isRunning();
    }

    public void resetAll() {
        Preferences.removeAll();
        PreferenceRegistry.resetAll();

        String timestamp = LocalDateTime.now()
            .format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss"));
        resetMessage.set("Preferences reset at " + timestamp);
    }
}
