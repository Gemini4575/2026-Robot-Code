package frc.robot.commands.shooter.base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class AutoShootBase extends Command {
    private final ShooterSubsystem s;
    public int Velocity = 6784;
    private final AdvancerSubsystem sadfd;
    private Timer timer = new Timer();

    public AutoShootBase(ShooterSubsystem a, AdvancerSubsystem ds, int velocity) {
        s = a;
        sadfd = ds;
        Velocity = velocity;
        addRequirements(a, ds);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (s.runShooterAtVelocity(Velocity)) {
            sadfd.advance();
        } else {
            sadfd.stopAdvancer();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(5);
    }

    @Override
    public void end(boolean interrupted) {
        s.stopShooter();
        sadfd.stopAdvancer();
    }

}
