package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommands {
    private ClimberCommands() {};


    public static Command setPosition(ClimberSubsystem climber, DoubleSupplier degrees) {
        return Commands.run(()->{
            climber.setPositionDegrees(degrees.getAsDouble());
        }, climber);
    }

    public static Command climberUp(ClimberSubsystem climber) {
        return Commands.run(()->{
            climber.setPositionDegrees(Constants.Climber.kMaxAngleDegrees);
        }, climber);
    }

    public static Command climberDown(ClimberSubsystem climber) {
        return Commands.run(()->{
            climber.setPositionDegrees(Constants.Climber.kMinAngleDegrees);
        }, climber);
    }
}
