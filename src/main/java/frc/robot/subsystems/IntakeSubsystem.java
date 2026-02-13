// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public SparkMax armMotor;
  public SparkMaxConfig armMotorConfig;
  public SparkClosedLoopController armController;
  public RelativeEncoder armEncoder;

  public SparkMax intakeMotor;
  public SparkMaxConfig intakeMotorConfig;
  public SparkClosedLoopController intakeController;
  public RelativeEncoder intakeEncoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    armMotor = new SparkMax(Constants.IntakeConstants.armMotorID, MotorType.kBrushless);
    armMotorConfig = new SparkMaxConfig();
    armController = armMotor.getClosedLoopController();
    armEncoder = armMotor.getEncoder();
    armMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeController = intakeMotor.getClosedLoopController();
    intakeEncoder = intakeMotor.getEncoder();
    intakeMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    
    armMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
