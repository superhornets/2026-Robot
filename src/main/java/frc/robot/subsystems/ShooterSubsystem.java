// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterSubsystem extends SubsystemBase {
  // HARDWARE OBJECTS

  // flywheel
  private SparkFlex flywheelMotorLeft;
  private SparkClosedLoopController flywheelControllerLeft;
  private SparkFlex flywheelMotorRight;
  private SparkClosedLoopController flywheelControllerRight;

  // other
  private SparkMax hoodMotor;
  private SparkClosedLoopController hoodController;
  private SparkFlex agitatorMotor;
  private SparkClosedLoopController agitatorController;
  private SparkFlex feederMotor;
  private SparkClosedLoopController feederController;

  // SIMULATION OBJECTS
  private SparkMaxSim hoodMotorSim;
  private SparkAbsoluteEncoderSim hoodEncoderSim;
  private DCMotor hoodGearboxSim;
  private SingleJointedArmSim hoodSim;
  private SparkFlexSim flywheelMotorSim;
  private FlywheelSim flywheelSim;
  private DCMotor flywheelGearboxSim;
  // Feeder simulation (matches flywheel)
  private SparkFlexSim feederMotorSim;
  private FlywheelSim feederSim;
  private DCMotor feederGearboxSim;
  // Agitator simulation (matches flywheel)
  private SparkFlexSim agitatorMotorSim;
  private FlywheelSim agitatorSim;
  private DCMotor agitatorGearboxSim;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Setup Motors and Controllers
    hoodMotor = new SparkMax(Constants.Shooter.CAN.kHood, MotorType.kBrushless);

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(15)
        .i(0)
        .d(0.5)
        .positionWrappingEnabled(false)
        .allowedClosedLoopError(Units.degreesToRotations(0.2), ClosedLoopSlot.kSlot0);
    //   .maxMotion
    //   .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
    //   .allowedProfileError(Units.degreesToRotations(0.2))
    //   .cruiseVelocity(120)
    //   .maxAcceleration(6_000.0, ClosedLoopSlot.kSlot0);

    hoodMotor.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    hoodController = hoodMotor.getClosedLoopController();

    flywheelMotorRight = new SparkFlex(Constants.Shooter.CAN.kFlywheelRight, MotorType.kBrushless);
    SparkFlexConfig flywheelConfigRight = new SparkFlexConfig();
    flywheelConfigRight
        .smartCurrentLimit(20, 40, 1000)
        .closedLoop
        .p(0.0002)
        .i(0)
        .d(0)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    flywheelMotorRight.configure(
        flywheelConfigRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    flywheelControllerRight = flywheelMotorRight.getClosedLoopController();

    flywheelMotorLeft = new SparkFlex(Constants.Shooter.CAN.kFlywheelLeft, MotorType.kBrushless);
    SparkFlexConfig flywheelConfigLeft = new SparkFlexConfig();
    flywheelConfigLeft
        .inverted(true)
        .smartCurrentLimit(20, 40, 1000)
        .closedLoop
        .p(0.0002)
        .i(0)
        .d(0)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    flywheelMotorLeft.configure(
        flywheelConfigLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    flywheelControllerLeft = flywheelMotorLeft.getClosedLoopController();

    feederMotor = new SparkFlex(Constants.Shooter.CAN.kFeeder, MotorType.kBrushless);
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.0006)
        .i(0)
        .d(0.001)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    feederMotor.configure(
        feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feederController = feederMotor.getClosedLoopController();

    agitatorMotor = new SparkFlex(Constants.Shooter.CAN.kAgitator, MotorType.kBrushless);
    SparkFlexConfig agigitatorConfig = new SparkFlexConfig();
    agigitatorConfig
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.0005)
        .i(0)
        .d(0.001)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    agitatorMotor.configure(
        agigitatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    agitatorController = agitatorMotor.getClosedLoopController();

    // SIMULATION OBJECTS
    hoodGearboxSim = DCMotor.getNEO(1);
    hoodMotorSim = new SparkMaxSim(hoodMotor, hoodGearboxSim);
    hoodEncoderSim = new SparkAbsoluteEncoderSim(hoodMotor);

    hoodSim =
        new SingleJointedArmSim(
            hoodGearboxSim,
            Constants.Shooter.SIM.kHoodGearRatio,
            1,
            1,
            Constants.Shooter.kHoodMaxAngle,
            Constants.Shooter.kHoodMinAngle,
            true,
            Constants.Shooter.kHoodMaxAngle);

    // Flywheel sim
    flywheelGearboxSim = DCMotor.getNeoVortex(2);
    flywheelMotorSim = new SparkFlexSim(flywheelMotorRight, flywheelGearboxSim);
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                flywheelGearboxSim,
                Constants.Shooter.SIM.kFlywheelMOI,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            flywheelGearboxSim);

    // Feeder sim
    feederGearboxSim = DCMotor.getNeoVortex(1);
    feederMotorSim = new SparkFlexSim(feederMotor, feederGearboxSim);
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                feederGearboxSim,
                Constants.Shooter.SIM.kFlywheelMOI,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            feederGearboxSim);

    // Agitator sim
    agitatorGearboxSim = DCMotor.getNeoVortex(1);
    agitatorMotorSim = new SparkFlexSim(agitatorMotor, agitatorGearboxSim);
    agitatorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                agitatorGearboxSim,
                Constants.Shooter.SIM.kFlywheelMOI * 10,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            agitatorGearboxSim);
  }

  public void periodic() {
    setHoodAngle(SmartDashboard.getNumber("Shooter/HoodAngle", 0));
  }

  public void update(Pose2d robotPose) {
    double dist = getDistance(robotPose);

    setShootDist(dist);
  }

  public double getDistance(Pose2d RobotPose) {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose3d hubCenter =
        isFlipped ? Constants.FieldConstants.RedHubCenter : Constants.FieldConstants.BlueHubCenter;

    double relativeHubX = hubCenter.getX() - RobotPose.getX();
    double relativeHubY = hubCenter.getY() - RobotPose.getY();
    System.out.println("relativeX: " + relativeHubX + ", relativeY: " + relativeHubY);
    // double HubZ = hubCenter.getZ();

    double dist = Math.sqrt(relativeHubX * relativeHubX + relativeHubY * relativeHubY);

    return dist;
  }

  public void setShootDist(double dist) {
    double maxDist = 1;
    double minDist = 0;

    double hoodAngleMin = 0;
    double hoodAngleMax = 45;

    double flywheelSpeedMin = 300;
    double flywheelSpeedMax = 4000;

    double ratio = (dist - minDist) / (maxDist - minDist);
    double hoodAngle =
        ratio * (hoodAngleMax - hoodAngleMin)
            + hoodAngleMin; // ratio * (maxValue - minValue) + minValue
    double flywheelSpeed = ratio * (flywheelSpeedMax - flywheelSpeedMin) + flywheelSpeedMin;

    setHoodAngle(hoodAngle);
    startFlywheel(flywheelSpeed);
  }

  public void setHoodAngle(double angleRotations) {
    hoodController.setSetpoint(angleRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void startFlywheel(double speedRPM) {
    double speedClamped =
        MathUtil.clamp(
            speedRPM, Constants.Shooter.kFlywheelMinSpeed, Constants.Shooter.kFlywheelMaxSpeed);
    flywheelControllerRight.setSetpoint(
        speedClamped, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    flywheelControllerLeft.setSetpoint(
        speedClamped, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    flywheelControllerRight.setSetpoint(
        speedClamped, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void stopFlywheel(boolean coast) {
    flywheelControllerRight.setSetpoint(
        0.0,
        coast ? ControlType.kDutyCycle : ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
    flywheelControllerLeft.setSetpoint(
        0.0,
        coast ? ControlType.kDutyCycle : ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
  }

  public void startFeeder() {
    feederController.setSetpoint(
        3000, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(
        1500, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void startReverseFeeder() {
    feederController.setSetpoint(-200, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(1000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    startFlywheel(-200);
  }

  public void stopReverseFeeder() {
    stopFeeder();
    stopFlywheel(false);
  }

  public void stopFeeder() {
    feederController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  @AutoLogOutput(key = "Shooter/hoodAngleLogged")
  public double getHoodAngle() {
    return hoodMotor.getAbsoluteEncoder().getPosition();
  }

  @AutoLogOutput(key = "Shooter/FlywheelVelocitySetpointRPM")
  public double getFlywheelVelocitySetpointRPM() {
    return flywheelControllerRight.getSetpoint();
  }

  @AutoLogOutput(key = "Shooter/FlywheelVelocityRPM")
  public double getFlywheelVelocityRPM() {
    return flywheelMotorRight.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Shooter/FlywheelLeftAmps")
  public double getLeftAmps() {
    return flywheelMotorLeft.getOutputCurrent();
  }

  @AutoLogOutput(key = "Shooter/FlywheelRightAmps")
  public double getRightAmps() {
    return flywheelMotorRight.getOutputCurrent();
  }

  @AutoLogOutput(key = "Shooter/FlywheelRightOutput")
  public double getRightOutput() {
    return flywheelMotorRight.getAppliedOutput();
  }

  public void simulationPeriodic() {
    // Hood
    hoodSim.setInput(hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    hoodSim.update(Constants.SIM.interval);
    hoodMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            hoodSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        Constants.SIM.interval);

    hoodEncoderSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(hoodSim.getVelocityRadPerSec())
            / Constants.Shooter.SIM.kHoodGearRatio,
        Constants.SIM.interval);

    // Flywheel
    flywheelMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            flywheelSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
        flywheelSim.setInput(2 * flywheelMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    flywheelSim.update(Constants.SIM.interval);

    // Feeder
    feederMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            feederSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
    feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    feederSim.update(Constants.SIM.interval);

    // Agitator
    agitatorMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            agitatorSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
    agitatorSim.setInput(agitatorMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    agitatorSim.update(Constants.SIM.interval);
  }
}
