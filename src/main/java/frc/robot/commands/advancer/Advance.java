package frc.robot.commands.advancer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;

public class Advance extends Command {
    private AdvancerSubsystem advancer;

    public Advance(AdvancerSubsystem advancer) {
        this.advancer = advancer;
        addRequirements(advancer);
    }

    @Override
    public void initialize() {
        advancer.advance();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        advancer.stopAdvancer();
    }
}