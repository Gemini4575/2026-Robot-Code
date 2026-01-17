package frc.robot.commands.advancer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topDeck.AdvancerSubsystem;

public class ReverseAdvancer extends Command {
    private final AdvancerSubsystem advancer;

    public ReverseAdvancer(AdvancerSubsystem advancerSubsystem) {
        this.advancer = advancerSubsystem;
        addRequirements(advancerSubsystem);
    }

    @Override
    public void execute() {
        advancer.reverse();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        advancer.stopAdvancer();
    }

    @Override
    public void initialize() {
    }
}
