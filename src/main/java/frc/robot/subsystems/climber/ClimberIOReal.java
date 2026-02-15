package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO
{
    public SparkMax motor = new SparkMax(Constants.CanIds.ClimberMotor1, MotorType.kBrushless);
    public SparkMaxConfig motorConfig = new SparkMaxConfig();
    public SparkClosedLoopController controller = motor.getClosedLoopController();
    public RelativeEncoder encoder = motor.getEncoder();  
    boolean extending = false;  

    public ClimberIOReal()
    {
        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void extendClimber(double targetPosition)
    {
        controller.setSetpoint(targetPosition, ControlType.kPosition);
        extending = true;
    }

    @Override
    public void retractClimber() 
    {
        controller.setSetpoint(0, ControlType.kPosition);
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
