package frc.robot.utilities.logging;

import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.utilities.logging.sendables.LoggableSendableBuilder;

/** A wrapper around DogLog to use for the sake of consistency and customizability */
public class HoundLog extends DogLog {
    private static HashMap<String, LoggableSendableBuilder> sendables = new HashMap<>();
    public static void log(String key, Sendable sendable) {
        if (sendables.containsKey(key)) {
            return;
        }
        LoggableSendableBuilder builder = new LoggableSendableBuilder(key);
        sendable.initSendable(builder);
        sendables.put(key, builder);
    }

    public static void updateSendables() {
        for (Map.Entry<String, LoggableSendableBuilder> entry : sendables.entrySet()) {
            entry.getValue().log(entry.getKey());
        }
    }
}
