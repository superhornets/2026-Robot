// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.util.preferences.PreferenceNumber;

public class IntakeIOHardware implements IntakeIO {
  private final SparkMax armMotor;
  private final SparkClosedLoopController armController;
  private final TalonFX rollerMotor;
  private final Servo servo1;
  private final Servo servo2;

  private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage rollerVelocity = new VelocityVoltage(0);

  private final PreferenceNumber rollerP;
  private final PreferenceNumber rollerD;

  public IntakeIOHardware(int armID, int rollerID) {
    servo1 = new Servo(Constants.Intake.kServo1Port);
    servo2 = new Servo(Constants.Intake.kServo2Port);

    armMotor = new SparkMax(armID, MotorType.kBrushless);
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(2)
        .i(0.0001)
        .d(50)
        .positionWrappingEnabled(false)
        .allowedClosedLoopError(Units.degreesToRotations(0.5), ClosedLoopSlot.kSlot0);
    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    armController = armMotor.getClosedLoopController();

    rollerP = new PreferenceNumber("Intake/RollerP", 0.0005);
    rollerD = new PreferenceNumber("Intake/RollerD", 0.0001);

    rollerMotor = new TalonFX(rollerID);
    configureRoller();
  }

  private void configureRoller() {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.Slot0.kP = rollerP.get();
    rollerConfig.Slot0.kD = rollerD.get();
    rollerConfig.Slot0.kV = 0.12;
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armPositionRotations = armMotor.getAbsoluteEncoder().getPosition();
    inputs.armAtSetpoint = armController.isAtSetpoint();
    inputs.rollerVelocityRPM = rollerMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  public void setArmPosition(double rotations) {
    armController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setServoAngles(double angle1, double angle2) {
    servo1.setAngle(angle1);
    servo2.setAngle(angle2);
  }

  @Override
  public void setRollerVelocity(double rpm) {
    rollerMotor.setControl(rollerVelocity.withVelocity(rpm / 60.0));
  }

  @Override
  public void stopRoller() {
    rollerMotor.setControl(rollerDutyCycle.withOutput(0.0));
  }
}
