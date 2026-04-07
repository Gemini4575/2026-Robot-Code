package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class ReverseShoot extends Command{
    private final ShooterSubsystem s;
    private final AdvancerSubsystem a;

    public ReverseShoot(ShooterSubsystem ss, AdvancerSubsystem aa){
        s = ss;
        a = aa;
        addRequirements(ss,aa);
    }

    @Override 
    public void execute(){
        s.reverse();
        a.reverse();
    }

    @Override
    public void end(boolean in){
        s.stopShooter();
        a.stopAdvancer();
    }
}
