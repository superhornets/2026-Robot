package frc.robot.Commands;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.Command;

public class OrchestraPlayCommand extends Command {
  // Declare subsystem variables
  private final Orchestra m_orchestra;

  public OrchestraPlayCommand(Orchestra orchestra) {
    m_orchestra = orchestra;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_orchestra.play();
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
