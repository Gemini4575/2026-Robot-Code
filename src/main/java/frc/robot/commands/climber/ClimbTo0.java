package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ClimberSubsystem;

public class ClimbTo0 extends Command {
    private ClimberSubsystem climber;

    public ClimbTo0(ClimberSubsystem climberSubsystem) {
        this.climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climber.MoveTo0();
    }

    @Override
    public boolean isFinished() {
        return climber.MoveTo0();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
    
}
