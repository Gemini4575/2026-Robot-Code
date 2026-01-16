package frc.robot.model;

public enum MetricName {

    // message for data capture start
    MESSAGE,

    // robot data
    LOCATION_ESTIMATE,
    AUTO_STATE,
    TELEOP_STATE,
    DISABLED_STATE,

    // overall drivetrain metrics
    REQUESTED_SPEED_X,
    REQUESTED_SPEED_Y,
    REQUESTED_ROTATION,

    // individual module metrics
    REQ_SPEED_M0,
    COMMANDED_SPEED_M0,
    REQ_TURN_M0,
    COMMANDED_TURN_M0,
    CURRENT_ANGLE_M0,
    ACTUAL_VELOCITY_M0,
    DRIVE_ENCODER_VALUE_M0,
    REQ_SPEED_M1,
    COMMANDED_SPEED_M1,
    REQ_TURN_M1,
    COMMANDED_TURN_M1,
    CURRENT_ANGLE_M1,
    ACTUAL_VELOCITY_M1,
    DRIVE_ENCODER_VALUE_M1,
    REQ_SPEED_M2,
    COMMANDED_SPEED_M2,
    REQ_TURN_M2,
    COMMANDED_TURN_M2,
    CURRENT_ANGLE_M2,
    ACTUAL_VELOCITY_M2,
    DRIVE_ENCODER_VALUE_M2,
    REQ_SPEED_M3,
    COMMANDED_SPEED_M3,
    REQ_TURN_M3,
    COMMANDED_TURN_M3,
    CURRENT_ANGLE_M3,
    ACTUAL_VELOCITY_M3,
    DRIVE_ENCODER_VALUE_M3;

    public static MetricName reqSpeed(int moduleIdx) {
        return MetricName.valueOf("REQ_SPEED_M" + moduleIdx);
    }

    public static MetricName commandedSpeed(int moduleIdx) {
        return MetricName.valueOf("COMMANDED_SPEED_M" + moduleIdx);
    }

    public static MetricName reqTurn(int moduleIdx) {
        return MetricName.valueOf("REQ_TURN_M" + moduleIdx);
    }

    public static MetricName commandedTurn(int moduleIdx) {
        return MetricName.valueOf("COMMANDED_TURN_M" + moduleIdx);
    }

    public static MetricName currentAngle(int moduleIdx) {
        return MetricName.valueOf("CURRENT_ANGLE_M" + moduleIdx);
    }

    public static MetricName actualVelocity(int moduleIdx) {
        return MetricName.valueOf("ACTUAL_VELOCITY_M" + moduleIdx);
    }

    public static MetricName driveEncoderValue(int moduleIdx) {
        return MetricName.valueOf("DRIVE_ENCODER_VALUE_M" + moduleIdx);
    }

    public static MetricName message() {
        return MetricName.valueOf("MESSAGE");
    }

}
