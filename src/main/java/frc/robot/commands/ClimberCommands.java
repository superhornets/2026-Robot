package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberCommands {
  private ClimberCommands() {}
  

  public static Command setPosition(ClimberSubsystem climber, DoubleSupplier degrees) {
    return Commands.run(
        () -> {
          climber.setPositionDegrees(degrees.getAsDouble());
        },
        climber);
  }

  public static Command climberUp(ClimberSubsystem climber) {
    return Commands.runOnce(
        () -> {
          climber.climberUp();
        },
        climber);
  }

  public static Command climberDown(ClimberSubsystem climber) {
    return Commands.runOnce(
        () -> {
          climber.climberDown();
        },
        climber);
  }

  public static Command zero(ClimberSubsystem climber) {
    return Commands.sequence(
      Commands.runOnce(() -> { climber.startZeroing(); }, climber),
      Commands.waitUntil(() -> { return climber.isStalled(); }),
      Commands.runOnce(() -> { climber.setZero(); }, climber)
    );
  }
}
