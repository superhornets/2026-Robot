package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SquareAutoCommand extends Command {
  // Declare subsystem variables
  private final Command m_command;

  public SquareAutoCommand(Command command) {
    m_command = command;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    CommandScheduler.getInstance().schedule(m_command);
  }

  // @Override
  // public void end(boolean interrupted) {
  //     // Stop command
  // }

  /*@Override
  public boolean isFinished() {
      // Have we reached our destination?
      return m_elevatorSubsystem.isAtSetpoint();
  }*/
}
