package frc.robot.commands.auto;

import static frc.robot.LocationData.REEF_BACK_ANY_TO_STATION_LEFT;
import static frc.robot.LocationData.REEF_FRONT_TO_STATION_LEFT;
import static frc.robot.LocationData.REEF_FRONT_TO_STATION_RIGHT;
import static frc.robot.LocationData.REEF_LEFT_TO_STATION_LEFT;
import static frc.robot.LocationData.REEF_RIGHT_TO_STATION_RIGHT;
import static frc.robot.LocationData.START_TO_REEF_FRONT;
import static frc.robot.LocationData.START_TO_REEF_FRONT_LEFT;
import static frc.robot.LocationData.START_TO_REEF_FRONT_RIGHT;
import static frc.robot.LocationData.STATION_ANY_TO_REEF_BACK_CENTER;
import static frc.robot.LocationData.STATION_LEFT_TO_REEF_BACK_LEFT;
import static frc.robot.LocationData.STATION_RIGHT_TO_REEF_BACK_RIGHT;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driving.DriveToLocation;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class AutoCommandFactory {

    private final DrivetrainIO drivetrainIO;
    private final LaserCan laserCan;

    public AutoCommandFactory(DrivetrainIO drivetrainIO, LaserCan laserCan) {
        this.drivetrainIO = drivetrainIO;
        this.laserCan = laserCan;
    }

    private Command dropOneCenter() {
        return new SequentialCommandGroup(new WaitCommand(3), drive(START_TO_REEF_FRONT));
    }

    private Command dropOneLeft() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_LEFT));
    }

    private Command dropOneRight() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_RIGHT));
    }

    private Command dropTwoFrontCenterBackLeft() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), drive(REEF_FRONT_TO_STATION_LEFT),
                drive(STATION_LEFT_TO_REEF_BACK_LEFT),
                drive(REEF_BACK_ANY_TO_STATION_LEFT));
    }

    private Command dropTwoFrontCenterBackCenterLeftStation() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), drive(REEF_FRONT_TO_STATION_LEFT),
                drive(STATION_ANY_TO_REEF_BACK_CENTER));
    }

    private Command dropTwoFrontLeftBackLeft() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_LEFT),
                drive(REEF_LEFT_TO_STATION_LEFT),
                drive(STATION_LEFT_TO_REEF_BACK_LEFT),
                drive(REEF_BACK_ANY_TO_STATION_LEFT));
    }

    private Command dropTwoFrontLeftBackCenter() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_LEFT),
                drive(REEF_LEFT_TO_STATION_LEFT),
                drive(STATION_ANY_TO_REEF_BACK_CENTER));
    }

    private Command dropTwoFrontCenterBackRight() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), drive(REEF_FRONT_TO_STATION_RIGHT),
                drive(STATION_RIGHT_TO_REEF_BACK_RIGHT));
    }

    private Command dropTwoFrontCenterBackCenterRightStation() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), drive(REEF_FRONT_TO_STATION_RIGHT),
                drive(STATION_ANY_TO_REEF_BACK_CENTER));
    }

    private Command dropTwoFrontRightBackRight() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_RIGHT),
                drive(REEF_RIGHT_TO_STATION_RIGHT),
                drive(STATION_RIGHT_TO_REEF_BACK_RIGHT));
    }

    private Command dropTwoFrontRightBackCenter() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_RIGHT),
                drive(REEF_RIGHT_TO_STATION_RIGHT),
                drive(STATION_ANY_TO_REEF_BACK_CENTER));
    }

    private Command drive(PathContainer path) {
        return new DriveToLocation(drivetrainIO, laserCan, path);
    }

    public SendableChooser<Command> generateAutoOptions() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("[1] Center", dropOneCenter());
        chooser.addOption("[1] Left", dropOneLeft());
        chooser.addOption("[1] Right", dropOneRight());
        chooser.addOption("[2] Center - Left Station - Left Back", dropTwoFrontCenterBackLeft());
        chooser.addOption("[2] Center - Left Station - Center Back", dropTwoFrontCenterBackCenterLeftStation());
        chooser.addOption("[2] Left - Left Station - Left Back", dropTwoFrontLeftBackLeft());
        chooser.addOption("[2] Left - Left Station - Center Back", dropTwoFrontLeftBackCenter());
        chooser.addOption("[2] Center - Right Station - Right Back", dropTwoFrontCenterBackRight());
        chooser.addOption("[2] Center - Right Station - Center Back", dropTwoFrontCenterBackCenterRightStation());
        chooser.addOption("[2] Right - Right Station - Right Back", dropTwoFrontRightBackRight());
        chooser.addOption("[2] Right - Right Station - Center Back", dropTwoFrontRightBackCenter());
        return chooser;
    }

}
