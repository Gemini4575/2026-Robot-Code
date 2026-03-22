package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class XTheWheels extends Command {
    private DrivetrainIO d;
    private Timer timer;

    public XTheWheels(DrivetrainIO d) {
        this.d = d;
        this.timer = new Timer();
        addRequirements(d);
    }

    @Override
    public void initialize() {
        timer.reset();
        d.XTheWheels();
        timer.start();
    }

    @Override
    public void execute() {
        d.XTheWheels();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3.0); // Run for 3 seconds
    }

}
