package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO 
{
    SparkMax controller = new SparkMax(Constants.CanIds.ClimberMotor1, MotorType.kBrushless);
    SparkMaxSim controllerSim = new SparkMaxSim(controller, DCMotor.getNEO(1));
    boolean extending = false;
    // TODO Set up an encoder sim too and add integrate that into the input update

    @Override
    public void extendClimber(double targetPosition)
    {
        controllerSim setpoint targetPosition
        extending = true;
    }

    @Override
    public void retractClimber()
    {
        controllerSim setpoint = 0, ControlType.kPosition
        extending = false;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        if(SmartDashboard.getBoolean("Reset Encoder", false))
        {
            SmartDashboard.putBoolean("ResetEncoder", false);
            encoder.setPosition(0);
        }

        inputs.extending = extending;
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
    }
}
