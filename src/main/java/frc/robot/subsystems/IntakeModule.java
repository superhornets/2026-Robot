// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeModule extends SubsystemBase {
  // HARDWARE OBJECTS
  private SparkMax armMotor;
  private SparkClosedLoopController armController;
  private SparkMax rollerMotor;
  private SparkClosedLoopController rollerController;
  
  // SIMULATION OBJECTS
  private SparkMaxSim armMotorSim;
  private DCMotor armGearboxSim;
  private SingleJointedArmSim armSim;
  private SparkMaxSim rollerMotorSim;
  private FlywheelSim rollerFlywheelSim;
  private DCMotor rollerGearboxSim;
  
  /** Creates a new IntakeModule. */
  public IntakeModule(int armID, int rollerID) {
    // Setup Motors and Controllers
    armMotor = new SparkMax(armID, MotorType.kBrushless);
    
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder);
    armConfig.closedLoop.p(0.1).i(0).d(0.01).maxMotion.maxAcceleration(Units.radiansPerSecondToRotationsPerMinute(Math.PI), ClosedLoopSlot.kSlot0);
    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    armController = armMotor.getClosedLoopController();
    
    rollerMotor = new SparkMax(rollerID, MotorType.kBrushless);
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.closedLoop.p(0.1).i(0).d(0.01).maxMotion.maxAcceleration(Math.PI * 10, ClosedLoopSlot.kSlot0);
    rollerMotor.configure(rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerController = rollerMotor.getClosedLoopController();
    
    // SIMULATION OBJECTS
    armGearboxSim = DCMotor.getNEO(1);
    armMotorSim = new SparkMaxSim(armMotor, armGearboxSim);
    
    armSim = new SingleJointedArmSim(
    armGearboxSim, 
    Constants.Intake.Sim.kArmGearRatio, 
    Constants.Intake.Sim.kArmMOI, 
    Constants.Intake.Sim.kArmLengthMeters, 
    Constants.Intake.kRaisedAngleRad, 
    Constants.Intake.kLoweredAngleRad, 
    true, 
    Constants.Intake.kRaisedAngleRad, 
    null);
    
    rollerGearboxSim = DCMotor.getNEO(1);
    rollerMotorSim = new SparkMaxSim(rollerMotor, rollerGearboxSim);
    
    rollerFlywheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
    rollerGearboxSim, 
    Constants.Intake.Sim.kRollerMOI, 
    Constants.Intake.Sim.kRollerGearRatio), 
    rollerGearboxSim);
  }
  
  public void periodic() {
    // Nothing to do here, as the subsystem is fully closed-loop in simulation and on the real robot. The only thing we need to do is update the simulation objects in simulationPeriodic().
  }
  
  /**
   * Lowers the arm and starts the roller at the intake speed.
   */
  public void lower() {
    armController.setSetpoint(Constants.Intake.kLoweredAngleRad, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    rollerController.setSetpoint(Constants.Intake.kIntakeRollerSpeed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }
  
  /**
   * Returns true if the arm is at the lowered setpoint.
   * @return true if the arm is at the lowered setpoint, false otherwise
   */
  public boolean isLowered() {
    return armController.getSetpoint() == Constants.Intake.kLoweredAngleRad && armController.isAtSetpoint();
  }
  
  /**
   * Raises the arm and stops the roller.
   */
  public void raise() {
    armController.setSetpoint(Constants.Intake.kRaisedAngleRad, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    rollerController.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  } 
  
  /**
   * Returns true if the arm is at the raised setpoint. Note that this does not check if the roller is stopped, as the roller may still be spinning due to inertia even after we set its setpoint to 0.
   * @return true if the arm is at the raised setpoint, false otherwise
   */
  public boolean isRaised() {
    return armController.getSetpoint() == Constants.Intake.kRaisedAngleRad && armController.isAtSetpoint();
  }
  
  public void simulationPeriodic() {
    // Arm
    armSim.setInput(armMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    armSim.update(Constants.SIM.interval);
    armMotorSim.iterate(
    Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
    armSim.getVelocityRadPerSec()),
    RoboRioSim.getVInVoltage(),
    Constants.SIM.interval);
    
    // Roller
    rollerFlywheelSim.setInput(rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    rollerFlywheelSim.update(Constants.SIM.interval);
    rollerMotorSim.iterate(
    Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
    rollerFlywheelSim.getAngularVelocityRadPerSec()),
    RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
    Constants.SIM.interval); // Time interval, in Seconds
    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(rollerFlywheelSim.getCurrentDrawAmps() + armSim.getCurrentDrawAmps())
    );
  }
}
