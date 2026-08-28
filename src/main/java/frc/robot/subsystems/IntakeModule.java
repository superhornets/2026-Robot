// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeModule extends SubsystemBase {
  boolean lowered = false;

  // HARDWARE OBJECTS
  private Servo servo1;
  private Servo servo2;
  private SparkMax armMotor;
  private SparkClosedLoopController armController;
  // private SparkMax rollerMotor;
  private final TalonFX rollerMotor;
  private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0); 
  private final VelocityVoltage rollerVelocity = new VelocityVoltage(0);
  // private SparkClosedLoopController rollerController;

  // SIMULATION OBJECTS
  private SparkMaxSim armMotorSim;
  private SparkAbsoluteEncoderSim armEncoderSim;
  private DCMotor armGearboxSim;
  private SingleJointedArmSim armSim;

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

    servo1 = new Servo(Constants.Intake.kServo1Port);
    servo2 = new Servo(Constants.Intake.kServo2Port);

    // Setup Motors and Controllers
    armMotor = new SparkMax(armID, MotorType.kBrushless);
SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .inverted(inverted)
        .smartCurrentLimit(40)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(2)
        .i(0.0001)
        .d(50)
        .positionWrappingEnabled(false)
        .allowedClosedLoopError(Units.degreesToRotations(0.5), ClosedLoopSlot.kSlot0);
        //.feedForward.kS(0.008).kCos(0.1);


    armMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    armController = armMotor.getClosedLoopController();


    rollerMotor = new TalonFX(rollerID, "");


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
  }

  private void configureRoller() {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40; // Protect the motor
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    var slot0 = rollerConfig.Slot0;
    slot0.kP = inverted ? rollerRightP.get() : rollerLeftP.get();
    slot0.kD = inverted ? rollerRightD.get() : rollerLeftD.get();
    // Krakens often need a tiny bit of kV (feedforward) to help reach speed
    slot0.kV = 0.12; 
    
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  /** Lowers the arm and starts the roller at the intake speed. */
  public void lower() {
    // ensure we have the latest roller PID values from the dashboard before lowering and starting the roller
    lowered = true;
    //configureRoller();
    servo1.setAngle(80);
    servo2.setAngle(10);
   armController.setSetpoint(
       Constants.Intake.kLoweredAngle,
       ControlType.kPosition,
       ClosedLoopSlot.kSlot0);
  }

  /**
   * Returns true if the arm is at the lowered setpoint.
   *
   * @return true if the arm is at the lowered setpoint, false otherwise
   */
  public boolean isLowered() {
    return lowered;
    // return armController.getSetpoint() == Constants.Intake.kLoweredAngle
    //     && armController.isAtSetpoint();
  }

  /** Raises the arm and stops the roller. */
  public void raise() {
    lowered = false;
    servo1.setAngle(0);
    servo2.setAngle(90);
    armController.setSetpoint(
        Constants.Intake.kRaisedAngle,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);

  }

  public boolean isAtSetpoint() {
    return armController.isAtSetpoint();
  }

  public void raiseHalf() {
    lowered = false;
    double angle = (Constants.Intake.kRaisedAngle + Constants.Intake.kLoweredAngle) * 0.5;
    armController.setSetpoint(
        angle,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  public void startRoller(boolean reverse) {
    double speed = (inverted ? rollerRightSpeed.get() : rollerLeftSpeed.get()) / 60;
    if (reverse) {
      speed *= -1;
    }
    rollerMotor.setControl(rollerVelocity.withVelocity(speed));
  }

  public void stopRoller() {
        // We set the roller to 0 using duty cycle control, as setting it to 0 using velocity control
    // will cause the motor to brake and stop the rollers rather than letting them coast to a stop.
    // rollerController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rollerMotor.setControl(rollerDutyCycle.withOutput(0.0));
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
    // return armController.getSetpoint() == Constants.Intake.kRaisedAngle
    //     && armController.isAtSetpoint();
    return !lowered;
  }

  @AutoLogOutput(key = "{logScope}/ArmAngleRotations")
  public double getAngleRotations() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  @AutoLogOutput(key = "{logScope}/RollerVelocityRPM")
  public double getRollerVelocityRPM() {
    // return rollerMotor.getEncoder().getVelocity();
    return rollerMotor.getVelocity().getValueAsDouble() * 60;
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
  }

  // Accessor for the logging scope
  public String getLogScope() {
    return logScope;
  }
}
