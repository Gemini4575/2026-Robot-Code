package frc.robot.commands.driving;

import java.time.LocalTime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class TimedTestDrive extends Command {

    private ShuffleboardTab driveTime_tab = Shuffleboard.getTab("TimedTestDrive");

    private GenericEntry driveTime_entry = driveTime_tab.add("Drive Time (ms)", 0)
            .withPosition(0, 0)
            .withSize(3, 1)
            .getEntry();
    private GenericEntry estimatedDistance_entry = driveTime_tab.add("Estimated Distance (m)", 0)
            .withPosition(0, 1)
            .withSize(3, 1)
            .getEntry();
    private GenericEntry startTime_entery = driveTime_tab.add("Start Time (ms)", "")
            .withPosition(0, 2)
            .withSize(3, 1)
            .getEntry();
    private GenericEntry endTime_entery = driveTime_tab.add("End Time (ms)", "")
            .withPosition(0, 3)
            .withSize(3, 1)
            .getEntry();

    private final long durationMillis;
    private final double speed;
    private final DrivetrainIO driveTrain;

    private long startTime;
    private Pose2d initialPose;

    public TimedTestDrive(DrivetrainIO driveTrain, long durationMillis, double speed) {
        this.durationMillis = durationMillis;
        this.speed = speed;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        startTime_entery.setString(LocalTime.now().toString());
        startTime = System.currentTimeMillis();
        initialPose = driveTrain.getPose();
    }

    @Override
    public void execute() {
        driveTrain.drive(-speed, 0, 0, false);
        driveTime_entry.setDouble(System.currentTimeMillis() - startTime);
        estimatedDistance_entry.setDouble(
                initialPose.getTranslation().getDistance(driveTrain.getPose().getTranslation()));
    }

    @Override
    public boolean isFinished() {
        var done = System.currentTimeMillis() - startTime >= durationMillis;
        if (done) {
            driveTrain.drive(0, 0, 0, false);
        }
        endTime_entery.setString(LocalTime.now().toString());
        return done;
    }

}
