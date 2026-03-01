// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberIO.*;

public class ClimberSubsystem extends SubsystemBase {
  ClimberIOInputs inputs = new ClimberIOInputs();
  ClimberIO io;
  double lastTarget = 0.0;

  /* Creates a new ClimberSubsystem. */
  public ClimberSubsystem(ClimberIO IOImplementation) 
  {
    SmartDashboard.setDefaultBoolean("ExtendingClimber", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    SmartDashboard.setDefaultNumber("Climber Target Position", 0);

    io = IOImplementation;
  }

  public void extendClimber()
  {
    lastTarget = SmartDashboard.getNumber("Climber Target Position", 0);
    io.extendClimber(lastTarget);
  }

  public void retractClimber()
  {
    io.retractClimber();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    SmartDashboard.putBoolean("Extending Climber", inputs.extending);
    SmartDashboard.putNumber("Climber Actual Position", inputs.position);
    SmartDashboard.putNumber("Climber Actual Velocity", inputs.velocity);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber Target Position", lastTarget);
  }

  @Override
  public void simulationPeriodic()
  {
    this.periodic();
  }
}
