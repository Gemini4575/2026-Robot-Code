package frc.robot.commands.driving;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class XTheWheels extends Command {
    private DrivetrainIO d;

    public XTheWheels(DrivetrainIO d) {
        this.d = d;
        addRequirements(d);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        d.XTheWheels();
    }

    @Override
    public boolean isFinished() {
        return false; // Run for 3 seconds
    }

}
