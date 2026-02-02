package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class Stop extends Command {
    
    DrivetrainIO d;
    boolean isFinished;
    Timer timer;

    public Stop(DrivetrainIO d) {
        this.d = d;
        addRequirements(d);
    }

    @Override
    public void initialize() {

        timer = new Timer();
        isFinished = false;
        timer.start();
    }
//3 9/16 3 3/4
    @Override
    public void execute() {
        d.drive(0, 0, 0, true);
        if (timer.advanceIfElapsed(2)) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
