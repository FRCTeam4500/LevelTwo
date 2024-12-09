package frc.robot.utilities.logging;

import dev.doglog.DogLog;

/** A wrapper around DogLog to use for the sake of consistency and customizability */
public class HoundLog extends DogLog {
    public static void log(String name, Loggable value) {
        value.log(name);
    }
}
