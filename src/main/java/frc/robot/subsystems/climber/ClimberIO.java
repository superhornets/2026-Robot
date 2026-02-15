package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.*;;

public interface ClimberIO 
{
    class ClimberIOInputs implements LoggableInputs
    {
        public boolean extending = false;
        public double position = 0.0;
        public double velocity = 0.0;


        public void toLog(LogTable table)
        {
            table.put("extending", extending);
            table.put("Position", position);
            table.put("Velocity", velocity);
        }

        public void fromLog(LogTable table)
        {
            table.get("extending");
            table.get("Position");
            table.get("Velocity");
        }
    }

    public void extendClimber(double targetPosition);
    public void retractClimber();
    public void updateInputs(ClimberIOInputs inputs);
}
