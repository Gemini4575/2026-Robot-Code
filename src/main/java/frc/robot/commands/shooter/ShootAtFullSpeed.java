package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.advancer.Advance;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.IntakeSubystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class ShootAtFullSpeed extends Command {
    private ShooterSubsystem s;
    private AdvancerSubsystem sadfd;

    public ShootAtFullSpeed(ShooterSubsystem a, AdvancerSubsystem ds) {
        s = a;
        sadfd = ds;
        addRequirements(a, ds);
    }

    @Override
    public void execute() {
        if (s.runShooterAtVelocity(6784)) {
            sadfd.advance();
        } else {
            sadfd.stopAdvancer();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
