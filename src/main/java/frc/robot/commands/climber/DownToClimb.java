package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ClimberSubsystem;

public class DownToClimb extends Command {
    private ClimberSubsystem climber;

    public DownToClimb(ClimberSubsystem climberSubsystem) {
        this.climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climber.MoveDownToClimb();
    }

    @Override
    public boolean isFinished() {
        return climber.MoveDownToClimb();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
    
}
