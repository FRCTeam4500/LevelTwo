package frc.robot.utilities.logging.sendables;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class LoggableSendableBuilder implements SendableBuilder, Loggable {
    private NetworkTable table;
    private String name;
    private ArrayList<Runnable> loggingCalls;

    public LoggableSendableBuilder(String sendableName) {
        this.name = sendableName;
        loggingCalls = new ArrayList<>();
        table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable(sendableName);
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void setSmartDashboardType(String type) {
        HoundLog.log(name + "/.type", type);
    }

    @Override
    public void setActuator(boolean value) {}

    @Override
    public void setSafeState(Runnable func) {}

    @Override
    public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.getAsBoolean()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getBoolean(false));
                }
            });
        }
    }

    @Override
    public void publishConstBoolean(String key, boolean value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addIntegerProperty(String key, LongSupplier getter, LongConsumer setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.getAsLong()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getInteger(0));
                }
            });
        }
    }

    @Override
    public void publishConstInteger(String key, long value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addFloatProperty(String key, FloatSupplier getter, FloatConsumer setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.getAsFloat()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getFloat(0));
                }
            });
        }
    }

    @Override
    public void publishConstFloat(String key, float value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.getAsDouble()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getDouble(0));
                }
            });
        }
    }

    @Override
    public void publishConstDouble(String key, double value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.get()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getString(""));
                }
            });
        }
    }

    @Override
    public void publishConstString(String key, String value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter, Consumer<boolean[]> setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.get()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getBooleanArray(new boolean[]{}));
                }
            });
        }
    }

    @Override
    public void publishConstBooleanArray(String key, boolean[] value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addIntegerArrayProperty(String key, Supplier<long[]> getter, Consumer<long[]> setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.get()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getIntegerArray(new long[]{}));
                }
            });
        }
    }

    @Override
    public void publishConstIntegerArray(String key, long[] value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addFloatArrayProperty(String key, Supplier<float[]> getter, Consumer<float[]> setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.get()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getFloatArray(new float[]{}));
                }
            });
        }
    }

    @Override
    public void publishConstFloatArray(String key, float[] value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addDoubleArrayProperty(String key, Supplier<double[]> getter, Consumer<double[]> setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.get()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getDoubleArray(new double[]{}));
                }
            });
        }
    }

    @Override
    public void publishConstDoubleArray(String key, double[] value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addStringArrayProperty(String key, Supplier<String[]> getter, Consumer<String[]> setter) {
        if (getter != null) {
            loggingCalls.add(() -> HoundLog.log(name + "/" + key, getter.get()));
        }
        if (setter != null) {
            loggingCalls.add(() -> {
                if (HoundLog.getOptions().ntPublish()) {
                    setter.accept(table.getEntry(key).getStringArray(new String[]{}));
                }
            });
        }
    }

    @Override
    public void publishConstStringArray(String key, String[] value) {
        HoundLog.log(name + "/" + key, value);
    }

    @Override
    public void addRawProperty(String key, String typeString, Supplier<byte[]> getter, Consumer<byte[]> setter) {}

    @Override
    public void publishConstRaw(String key, String typeString, byte[] value) {}

    @Override
    public BackendKind getBackendKind() {
        return BackendKind.kUnknown;
    }

    @Override
    public boolean isPublished() {
        return true;
    }

    @Override
    public void update() {}

    @Override
    public void clearProperties() {
        loggingCalls.clear();
    }

    @Override
    public void addCloseable(AutoCloseable closeable) {}

    @Override
    public void log(String name) {
        for (Runnable call : loggingCalls) {
            call.run();
        }
    }
    
}
