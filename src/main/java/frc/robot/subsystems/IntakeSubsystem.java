// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  // Provide a logScope to each IntakeModule so the module knows which side it
  // represents and where to emit logs (e.g. "Intake.Left").
  private IntakeModule rightIntake =
      new IntakeModule(
          Constants.Intake.CAN.kRightArm, Constants.Intake.CAN.kRightRoller, true, "Intake/Right");

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
 
    SmartDashboard.setDefaultNumber("RightAngle", 0);
    SmartDashboard.setDefaultBoolean("RightRaised", false);
    SmartDashboard.setDefaultBoolean("LowerRight", false);
  }

  public void raiseAll() {
    rightIntake.raise();
  }



  public void lowerRight() {
    rightIntake.lower();
  }

  @Override
  public void simulationPeriodic() {

    boolean right = SmartDashboard.getBoolean("LowerRight", false);
   

    if (right) lowerRight();

  
    SmartDashboard.putBoolean("RightRaised", !rightIntake.isLowered());


    rightIntake.simulationPeriodic();
  }
}
