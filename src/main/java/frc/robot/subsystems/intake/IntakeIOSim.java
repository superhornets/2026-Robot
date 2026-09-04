// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          IntakeConstants.SIM.kArmGearRatio,
          IntakeConstants.SIM.kArmMOI,
          IntakeConstants.SIM.kArmLengthMeters,
          Units.rotationsToRadians(IntakeConstants.kRaisedAngle),
          Units.rotationsToRadians(IntakeConstants.kLoweredAngle),
          true,
          Units.rotationsToRadians(IntakeConstants.kRaisedAngle));

  private double armAppliedOutput = 0.0;
  private double armSetpoint = IntakeConstants.kRaisedAngle;

  // Roller: Kraken X60 (TalonFX), 1:1 gear ratio
  private final DCMotor rollerGearbox = DCMotor.getKrakenX60(1);
  private final FlywheelSim rollerSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          rollerGearbox,
          IntakeConstants.SIM.kRollerMOI,
          IntakeConstants.SIM.kRollerGearRatio),
      rollerGearbox);
  private double rollerAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double batteryVolts = RoboRioSim.getVInVoltage();

    armSim.setInput(armAppliedOutput * batteryVolts);
    armSim.update(Constants.SIM.interval);
    inputs.armPositionRotations = Units.radiansToRotations(armSim.getAngleRads());
    inputs.armAtSetpoint =
        Math.abs(inputs.armPositionRotations - armSetpoint) < Units.degreesToRotations(0.5);
    inputs.armCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());

    // Roller
    rollerSim.setInput(MathUtil.clamp(rollerAppliedVolts, -batteryVolts, batteryVolts));
    rollerSim.update(Constants.SIM.interval);
    inputs.rollerVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(rollerSim.getAngularVelocityRadPerSec());
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
  }

  @Override
  public void setArmPosition(double rotations) {
    armSetpoint = rotations;
    double error = rotations - Units.radiansToRotations(armSim.getAngleRads());
    armAppliedOutput = MathUtil.clamp(error * 5.0, -1.0, 1.0);
  }

  @Override
  public void setServoAngles(double angle1, double angle2) {
    // No servo simulation needed
  }

  @Override
  public void setRollerVelocity(double rpm) {
    // Kraken X60 free speed ~6000 RPM at 12V (matches hardware kV = 0.12 V·s/rot)
    rollerAppliedVolts = MathUtil.clamp((rpm / 6000.0) * 12.0, -12.0, 12.0);
  }

  @Override
  public void stopRoller() {
    rollerAppliedVolts = 0.0;
  }
}
