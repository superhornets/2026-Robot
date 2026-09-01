// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkFlex flywheelMotorLeft;
  private final SparkClosedLoopController flywheelControllerLeft;
  private final SparkFlex flywheelMotorRight;
  private final SparkClosedLoopController flywheelControllerRight;

  private final SparkMax hoodMotor;
  private final SparkClosedLoopController hoodController;
  private final SparkFlex spindexerMotor;
  private final SparkClosedLoopController spindexerController;
  private final SparkFlex feederMotor;
  private final SparkClosedLoopController feederController;

  private final LoggedNetworkNumber flywheelP = new LoggedNetworkNumber("Shooter/FlywheelP", 0.0007);
  private final LoggedNetworkNumber flywheelD = new LoggedNetworkNumber("Shooter/FlywheelD", 0.0000);

  public ShooterIOSparkMax() {
    hoodMotor = new SparkMax(Constants.Shooter.CAN.kHood, MotorType.kBrushless);
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(.1)
        .i(0)
        .d(0.01)
        .outputRange(-1, 1)
        .allowedClosedLoopError(Units.degreesToRotations(5), ClosedLoopSlot.kSlot0);
    hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    hoodController = hoodMotor.getClosedLoopController();
    hoodMotor.getEncoder().setPosition(0);

    // Flywheel
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig
        .smartCurrentLimit(80, 40, 1000)
        .closedLoop
        .p(flywheelP.get())
        .i(0)
        .d(flywheelD.get())
        .feedForward
        .kV(0.00177);
    flywheelConfig.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);
    flywheelConfig.limitSwitch.apply(
        new LimitSwitchConfig()
            .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
            .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor));

    flywheelMotorRight = new SparkFlex(Constants.Shooter.CAN.kFlywheelRight, MotorType.kBrushless);
    flywheelMotorRight.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelControllerRight = flywheelMotorRight.getClosedLoopController();

    flywheelMotorLeft = new SparkFlex(Constants.Shooter.CAN.kFlywheelLeft, MotorType.kBrushless);
    flywheelMotorLeft.configure(
        flywheelConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelControllerLeft = flywheelMotorLeft.getClosedLoopController();

    // Feeder
    feederMotor = new SparkFlex(Constants.Shooter.CAN.kFeeder, MotorType.kBrushless);
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.0006)
        .i(0)
        .d(0.001);
    feederMotor.configure(feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feederController = feederMotor.getClosedLoopController();

    // Spindexer
    spindexerMotor = new SparkFlex(Constants.Shooter.CAN.kSpindexer, MotorType.kBrushless);
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.0001)
        .i(0)
        .d(0.0001);
    spindexerConfig.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);
    spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spindexerController = spindexerMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelVelocityRPM = flywheelMotorRight.getEncoder().getVelocity();
    inputs.flywheelAtSetpoint = flywheelControllerRight.isAtSetpoint();

    inputs.hoodPositionRotations = hoodMotor.getEncoder().getPosition();
    inputs.hoodAtSetpoint = hoodController.isAtSetpoint();
    inputs.hoodCurrentAmps = hoodMotor.getOutputCurrent();
    inputs.hoodStalled = (inputs.hoodCurrentAmps > 8.0) && (Math.abs(hoodMotor.getEncoder().getVelocity()) < 5.0);

    inputs.feederVelocityRPM = feederMotor.getEncoder().getVelocity();
    inputs.spindexerVelocityRPM = spindexerMotor.getEncoder().getVelocity();
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelControllerLeft.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    flywheelControllerRight.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stopFlywheel(boolean coast) {
    ControlType type = coast ? ControlType.kDutyCycle : ControlType.kVelocity;
    flywheelControllerLeft.setSetpoint(0.0, type, ClosedLoopSlot.kSlot0);
    flywheelControllerRight.setSetpoint(0.0, type, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHoodPosition(double rotations) {
    hoodController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHoodOutput(double output) {
    hoodMotor.set(output);
  }

  @Override
  public void resetHoodEncoder() {
    hoodMotor.getEncoder().setPosition(0.0);
  }

  @Override
  public void setFeederVelocity(double rpm) {
    feederController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setFeederOutput(double output) {
    feederController.setSetpoint(output, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setSpindexerVelocity(double rpm) {
    spindexerController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setSpindexerOutput(double output) {
    spindexerController.setSetpoint(output, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }
}
