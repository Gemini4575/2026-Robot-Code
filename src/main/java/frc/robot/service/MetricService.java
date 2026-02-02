package frc.robot.service;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import frc.robot.model.MetricData;
import frc.robot.model.MetricName;

public class MetricService {

    private static final int MAX_CACHE_SIZE = 5000;
    private static List<MetricData> capturedMetrics = new ArrayList<>();
    private static StringArrayPublisher jsonPublisher;

    public static void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("metrics");
        StringArrayTopic jsonTopic = table.getStringArrayTopic("json");
        jsonPublisher = jsonTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        System.out.println("Topics: " + table.getTopics());
    }

    public static void close() {
        if (jsonPublisher != null) {
            jsonPublisher.close();
        }
    }

    public static void publish(MetricName metricName, double value) {
        publish(metricName, String.valueOf(value));
    }

    public static void publish(MetricName metricName, String value) {
        if (capturedMetrics.size() < MAX_CACHE_SIZE) {
            capturedMetrics.add(new MetricData(metricName, value, System.currentTimeMillis()));
        }
    }

    public static void publishRobotLocation(Pose2d location) {
        String value = String.format("%f;%f;%f",
                location.getX(), location.getY(), location.getRotation().getRadians());
        publish(MetricName.LOCATION_ESTIMATE, value);
    }

    private static String toJson(MetricData metricData) {
        return String.format("{\"metricName\": \"%s\", \"type\": \"double\", \"value\": \"%s\", \"timestamp\": %d}",
                metricData.metricName(),
                metricData.value(), metricData.timestamp());
    }

    public static void periodic() {
        if (capturedMetrics.isEmpty()) {
            return;
        }
        var previousList = capturedMetrics;
        capturedMetrics = new ArrayList<>();
        if (jsonPublisher != null) {
            // System.out.println("Publishing metric: " + metricName + " = " + value);
            jsonPublisher
                    .set(previousList.stream().map(MetricService::toJson).toArray(String[]::new));
        }
    }

}
