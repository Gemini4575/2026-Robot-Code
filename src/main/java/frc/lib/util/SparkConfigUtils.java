package frc.lib.util;

import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.spark.SparkMax;

public class SparkConfigUtils {

    public static void printConfig(SparkMax spark, FileWriter fileWriter) {
        try {
            fileWriter.write("absoluteEncoder\n");
            printGetterValues(fileWriter, spark.configAccessor.absoluteEncoder);
            fileWriter.write("alternateEncoder\n");
            printGetterValues(fileWriter, spark.configAccessor.alternateEncoder);
            fileWriter.write("analogSensor\n");
            printGetterValues(fileWriter, spark.configAccessor.analogSensor);
            fileWriter.write("closedLoop\n");
            printGetterValues(fileWriter, spark.configAccessor.closedLoop);
            fileWriter.write("encoder\n");
            printGetterValues(fileWriter, spark.configAccessor.encoder);
            fileWriter.write("limitSwitch\n");
            printGetterValues(fileWriter, spark.configAccessor.limitSwitch);
            fileWriter.write("signals\n");
            printGetterValues(fileWriter, spark.configAccessor.signals);
            fileWriter.write("softLimit\n");
            printGetterValues(fileWriter, spark.configAccessor.softLimit);
            fileWriter.write("configAccessor\n");
            printGetterValues(fileWriter, spark.configAccessor);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void printGetterValues(FileWriter fileWriter, Object obj) {
        for (var m : obj.getClass().getMethods()) {
            if (m.canAccess(obj) && m.getName().startsWith("get") && m.getParameterTypes().length == 0) {
                try {
                    var value = m.invoke(obj);
                    fileWriter.write(m.getName() + " = " + value + "\n");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

}
