// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class RebuiltMatch {
    
    @AutoLogOutput(key = "Match/IsHubActive")
    public boolean isOurHubActive() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        
        // Always active in Auto or End Game
        if (matchTime > 130 || matchTime <= 30) return true;
        
        boolean redInactiveFirst = gameData.equals("R");
        
        // Determine if our alliance is the one that starts "Inactive"
        boolean weAreInactiveFirst = (alliance.get() == Alliance.Red) ? redInactiveFirst : !redInactiveFirst;
        
        // Logic for the 4 Teleop Shifts
        if (matchTime > 105) return !weAreInactiveFirst; // Shift 1
        if (matchTime > 80)  return weAreInactiveFirst;  // Shift 2
        if (matchTime > 55)  return !weAreInactiveFirst; // Shift 3
        return weAreInactiveFirst;                       // Shift 4
    }
    
    @AutoLogOutput(key = "Match/HubDisplayColor")
    public String getHubDisplayColor() {
        double matchTime = DriverStation.getMatchTime();
        boolean active = isOurHubActive(); // Your existing logic
        var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        
        // 1. If it's inactive, keep it Black
        if (!active) return "#000000";
        
        // 2. Check for the 3-second "Warning" window
        // Shifts end at 105s, 80s, 55s, and 30s.
        boolean isWarningZone = (matchTime > 105 && matchTime < 108) || 
        (matchTime > 80 && matchTime < 83)   || 
        (matchTime > 55 && matchTime < 58);
        
        if (isWarningZone) {
            // Toggle color every 0.25 seconds for a "pulse" effect
            boolean blink = (Timer.getFPGATimestamp() % 0.5) > 0.25;
            if (blink) return "#000000"; 
        }
        
        // 3. Otherwise, show solid Alliance color
        return (alliance == Alliance.Red) ? "#FF0000" : "#0000FF";
    }
    
    
    public String updateHubStatusColor() {
        boolean active = isOurHubActive(); // Using the logic from our previous discussion
        var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        
        Color displayColor;
        if (!active) {
            displayColor = Color.kBlack; // Hub is Inactive
        } else {
            displayColor = (alliance == Alliance.Red) ? Color.kRed : Color.kBlue;
        }
        
        return displayColor.toHexString();
    }
    
}