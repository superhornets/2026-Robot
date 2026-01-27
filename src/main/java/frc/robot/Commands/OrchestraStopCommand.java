package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.Orchestra;

public class OrchestraStopCommand extends Command{
    //Declare subsystem variables
    private final Orchestra m_orchestra;

    public OrchestraStopCommand(Orchestra orchestra) {
        m_orchestra = orchestra;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_orchestra.stop();
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
