// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Used for @AutoLogOutput key interpolation (e.g. "Intake/Right/isRaised")
  private final String logScope;

  private boolean lowered = false;
  private double armSetpointRotations = IntakeConstants.kRaisedAngle;

  private final Mechanism2d m_mechanism = new Mechanism2d(1, 1);
  private final MechanismRoot2d m_root = m_mechanism.getRoot("Intake", 0.1, 0.5);
  private final MechanismLigament2d m_arm =
      m_root.append(new MechanismLigament2d("Arm", IntakeConstants.SIM.kArmLengthMeters,
          Units.rotationsToDegrees(IntakeConstants.kRaisedAngle), 6, new Color8Bit(Color.kYellow)));
  private final MechanismLigament2d m_armSetpoint =
      m_root.append(new MechanismLigament2d("ArmSetpoint", IntakeConstants.SIM.kArmLengthMeters,
          Units.rotationsToDegrees(IntakeConstants.kRaisedAngle), 2, new Color8Bit(Color.kGray)));

  private final LoggedNetworkNumber rollerSpeed;

  public IntakeSubsystem(IntakeIO io, String logScope, double defaultRollerSpeedRPM) {
    this.io = io;
    this.logScope = logScope;
    this.rollerSpeed = new LoggedNetworkNumber(logScope + "/RollerSpeedRPM", defaultRollerSpeedRPM);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logScope, inputs);

    m_arm.setAngle(Units.rotationsToDegrees(inputs.armPositionRotations));
    m_armSetpoint.setAngle(Units.rotationsToDegrees(armSetpointRotations));
    Logger.recordOutput(logScope + "/Mechanism", m_mechanism);
  }

  /** Lowers the arm and prepares servos for intaking. */
  public void lower() {
    lowered = true;
    armSetpointRotations = IntakeConstants.kLoweredAngle;
    io.setServoAngles(80, 10);
    io.setArmPosition(IntakeConstants.kLoweredAngle);
  }

  /** Raises the arm and returns servos to stowed position. */
  public void raise() {
    lowered = false;
    armSetpointRotations = IntakeConstants.kRaisedAngle;
    io.setServoAngles(0, 90);
    io.setArmPosition(IntakeConstants.kRaisedAngle);
  }

  public void raiseHalf() {
    lowered = false;
    double angle = (IntakeConstants.kRaisedAngle + IntakeConstants.kLoweredAngle) * 0.5;
    armSetpointRotations = angle;
    io.setArmPosition(angle);
  }

  public void startRoller(boolean reverse) {
    double speed = rollerSpeed.get();
    io.setRollerVelocity(reverse ? -speed : speed);
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public boolean isLowered() {
    return lowered;
  }

  public boolean isAtSetpoint() {
    return inputs.armAtSetpoint;
  }

  @AutoLogOutput(key = "{logScope}/isRaised")
  public boolean isRaised() {
    return !lowered;
  }

  @AutoLogOutput(key = "{logScope}/ArmAngleRotations")
  public double getAngleRotations() {
    return inputs.armPositionRotations;
  }

  @AutoLogOutput(key = "{logScope}/RollerVelocityRPM")
  public double getRollerVelocityRPM() {
    return inputs.rollerVelocityRPM;
  }
}
