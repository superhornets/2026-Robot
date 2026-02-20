// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private IntakeModule leftIntake =
      new IntakeModule(Constants.Intake.CAN.kLeftArm, Constants.Intake.CAN.kLeftRoller);
  private IntakeModule rightIntake =
      new IntakeModule(Constants.Intake.CAN.kRightArm, Constants.Intake.CAN.kRightRoller);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    SmartDashboard.setDefaultNumber("LeftAngle", 0);
    SmartDashboard.setDefaultNumber("RightAngle", 0);
    SmartDashboard.setDefaultBoolean("LeftRaised", false);
    SmartDashboard.setDefaultBoolean("RightRaised", false);
    SmartDashboard.setDefaultBoolean("LowerLeft", false);
    SmartDashboard.setDefaultBoolean("LowerRight", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void raiseAll() {
    leftIntake.raise();
    rightIntake.raise();
  }

  public void lowerLeft() {
    leftIntake.lower();
    rightIntake.raise();
  }

  public void lowerRight() {
    leftIntake.raise();
    rightIntake.lower();
  }

  @Override
  public void simulationPeriodic() {

    boolean right = SmartDashboard.getBoolean("LowerRight", false);
    boolean left = SmartDashboard.getBoolean("LowerLeft", false);
    if (left) lowerLeft();
    if (right) lowerRight();

    SmartDashboard.putBoolean("LeftRaised", !leftIntake.isLowered());
    SmartDashboard.putBoolean("RightRaised", !rightIntake.isLowered());

    // double leftAngle = SmartDashboard.getNumber("LeftAngle", false);

    //    SmartDashboard.putBoolean("LeftRaised", true);
    // This method will be called once per scheduler run during simulation

    leftIntake.simulationPeriodic();
    rightIntake.simulationPeriodic();
  }
}
