// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeModule extends SubsystemBase {
  // HARDWARE OBJECTS
  private SparkMax armMotor;
  private SparkClosedLoopController armController;
  private SparkMax rollerMotor;
  private SparkClosedLoopController rollerController;

  // SIMULATION OBJECTS
  private SparkMaxSim armMotorSim;
  private SparkAbsoluteEncoderSim armEncoderSim;
  private DCMotor armGearboxSim;
  private SingleJointedArmSim armSim;
  private SparkMaxSim rollerMotorSim;
  private FlywheelSim rollerFlywheelSim;
  private DCMotor rollerGearboxSim;

  // Logging scope/prefix for this module (e.g. "Intake.Left")
  private final String logScope;

  private boolean inverted;
  private LoggedNetworkNumber rollerLeftSpeed = new LoggedNetworkNumber("/Tuning/ntake/Left/RollerSpeed", Constants.Intake.kIntakeRollerSpeedLeft);
  private LoggedNetworkNumber rollerLeftP = new LoggedNetworkNumber("/Tuning/ntake/Left/RollerP", 0.0005);
  private LoggedNetworkNumber rollerLeftD = new LoggedNetworkNumber("/Tuning/ntake/Left/RollerD", 0.0001);

  private LoggedNetworkNumber rollerRightSpeed = new LoggedNetworkNumber("/Tuning/ntake/Right/RollerSpeed", Constants.Intake.kIntakeRollerSpeedRight);
  private LoggedNetworkNumber rollerRightP = new LoggedNetworkNumber("/Tuning/ntake/Right/RollerP", 0.0005);
  private LoggedNetworkNumber rollerRightD = new LoggedNetworkNumber("/Tuning/ntake/Right/RollerD", 0.0001);


  /** Creates a new IntakeModule. */
  public IntakeModule(int armID, int rollerID, boolean inverted, String logScope) {
    this.logScope = logScope;
    this.inverted = inverted;


    // Setup Motors and Controllers
    armMotor = new SparkMax(armID, MotorType.kBrushless);
SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .inverted(inverted)
        .smartCurrentLimit(20)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(8)
        .i(0)
        .d(0.1)
        .positionWrappingEnabled(false)
        .allowedClosedLoopError(Units.degreesToRotations(0.2), ClosedLoopSlot.kSlot0)
        .maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedProfileError(Units.degreesToRotations(0.2))
        .cruiseVelocity(120)
        .maxAcceleration(6_000.0, ClosedLoopSlot.kSlot0);

    armMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    armController = armMotor.getClosedLoopController();

    
    rollerMotor = new SparkMax(rollerID, MotorType.kBrushless);

    configureRoller();

    // SIMULATION OBJECTS
    armGearboxSim = DCMotor.getNEO(1);
    armMotorSim = new SparkMaxSim(armMotor, armGearboxSim);
    armEncoderSim = new SparkAbsoluteEncoderSim(armMotor);

    armSim =
        new SingleJointedArmSim(
            armGearboxSim,
            Constants.Intake.SIM.kArmGearRatio,
            Constants.Intake.SIM.kArmMOI,
            Constants.Intake.SIM.kArmLengthMeters,
            Units.rotationsToRadians(Constants.Intake.kRaisedAngle),
            Units.rotationsToRadians(Constants.Intake.kLoweredAngle),
            true,
            Units.rotationsToRadians(Constants.Intake.kRaisedAngle));

    rollerGearboxSim = DCMotor.getNEO(1);
    rollerMotorSim = new SparkMaxSim(rollerMotor, rollerGearboxSim);

    rollerFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                rollerGearboxSim,
                Constants.Intake.SIM.kRollerMOI,
                Constants.Intake.SIM.kRollerGearRatio),
            rollerGearboxSim);
  }

  private void configureRoller() {
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(inverted ? rollerRightP.get() : rollerLeftP.get())
        .i(0)
        .d(inverted ? rollerRightD.get() : rollerLeftD.get())
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    rollerMotor.configure(
        rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerController = rollerMotor.getClosedLoopController();

  }

  /** Lowers the arm and starts the roller at the intake speed. */
  public void lower() {
    // ensure we have the latest roller PID values from the dashboard before lowering and starting the roller
    configureRoller();

    armController.setSetpoint(
        Constants.Intake.kLoweredAngle,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
    rollerController.setSetpoint(
        inverted ? rollerRightSpeed.get() : rollerLeftSpeed.get(),
        ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
  }

  /**
   * Returns true if the arm is at the lowered setpoint.
   *
   * @return true if the arm is at the lowered setpoint, false otherwise
   */
  public boolean isLowered() {
    return armController.getSetpoint() == Constants.Intake.kLoweredAngle
        && armController.isAtSetpoint();
  }

  /** Raises the arm and stops the roller. */
  public void raise() {
    armController.setSetpoint(
        Constants.Intake.kRaisedAngle,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
    // We set the roller to 0 using duty cycle control, as setting it to 0 using velocity control
    // will cause the motor to brake and stop the rollers rather than letting them coast to a stop.
    rollerController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  /**
   * Returns true if the arm is at the raised setpoint. Note that this does not check if the roller
   * is stopped, as the roller may still be spinning due to inertia even after we set its setpoint
   * to 0.
   *
   * @return true if the arm is at the raised setpoint, false otherwise
   */
  @AutoLogOutput(key = "{logScope}/isRaised")
  public boolean isRaised() {
    return armController.getSetpoint() == Constants.Intake.kRaisedAngle
        && armController.isAtSetpoint();
  }

  @AutoLogOutput(key = "{logScope}/ArmAngleRotations")
  public double getAngleRotations() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  @AutoLogOutput(key = "{logScope}/RollerVelocityRPM")
  public double getRollerVelocityRPM() {
    return rollerMotor.getEncoder().getVelocity();
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

    armEncoderSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec())
            / Constants.Intake.SIM.kArmGearRatio,
        Constants.SIM.interval);

    // Roller
    rollerFlywheelSim.setInput(rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    rollerFlywheelSim.update(Constants.SIM.interval);
    rollerMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            rollerFlywheelSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
  }

  // Accessor for the logging scope
  public String getLogScope() {
    return logScope;
  }
}
