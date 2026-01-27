package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class SquareAutoCommand extends Command {
    //Declare subsystem variables
    private final Command m_command;

    public SquareAutoCommand(Command command) {
        m_command = command;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_command.schedule();
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
