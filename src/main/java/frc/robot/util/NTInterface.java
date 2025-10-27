package frc.robot.util;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

public class NTInterface {
    private static ArrayList<String> registeredDashboards = new ArrayList<String>();
    private static ArrayList<String> dashboardData = new ArrayList<String>();

    // Initialize the NetworkTables interface
    static {
        var inst = edu.wpi.first.networktables.NetworkTableInstance.getDefault();
        var table = inst.getTable("Shambots_NTI");
        table.putValue("iface_hostname", NetworkTableValue.makeString("5907_1"));
        table.addListener(EnumSet.of(Kind.kValueAll), checkDashboard());
        System.out.println("NTInterface initialized");
    }

    private static TableEventListener checkDashboard() {
        return (table, key, event) -> {
            if(key.startsWith("cli_<5907_1>")){
                String client_name = key.substring(13, key.length() - 1);
                if(!registeredDashboards.contains(client_name)){
                    registeredDashboards.add(client_name);
                    dashboardData.add("");
                    System.out.println("Dashboard connected: " + client_name);
                }
                int index = registeredDashboards.indexOf(client_name);
                dashboardData.set(index, event.valueData.value.getString());
            }
        };
    }

    public static boolean isDashboardConnected(String dashboardName) {
        return registeredDashboards.contains(dashboardName);
    }

    public static NTITable getDashboardTable(String dashboardName) {
        return new NTITable(dashboardName);
    }

    public static class NTITable {
        private final String dashboardName;

        public NTITable(String dashboardName) {
            if (!isDashboardConnected(dashboardName)) {
                throw new IllegalArgumentException("Dashboard not connected: " + dashboardName);
            }
            this.dashboardName = dashboardName;
        }

        public NTIValue getValue(String key) {
            int index = registeredDashboards.indexOf(dashboardName);
            String rawData = dashboardData.get(index);
            String[] entries = rawData.split(";");
            for (String entry : entries) {
                if (entry.startsWith(key + "<")) {
                    return new NTIValue(dashboardName, entry);
                }
            }
            throw new IllegalArgumentException("Key not found: " + key);
        }

        public String getDashboardName() {
            return dashboardName;
        }

        public boolean valueExists(String key) {
            int index = registeredDashboards.indexOf(dashboardName);
            String rawData = dashboardData.get(index);
            String[] entries = rawData.split(";");
            for (String entry : entries) {
                if (entry.startsWith(key + "<")) {
                    return true;
                }
            }
            return false;
        }

        public List<String> getAllKeys() {
            int index = registeredDashboards.indexOf(dashboardName);
            String rawData = dashboardData.get(index);
            String[] entries = rawData.split(";");
            List<String> keys = new ArrayList<>();
            for (String entry : entries) {
                String key = entry.split("<")[0];
                keys.add(key);
            }
            return keys;
        }
    }

    public static class NTIValue {
        private final String dashboardName;
        private final String stringifiedData;
        private final String key;
        private final types type;

        public NTIValue(String dashboardName, String rawData) {
            this.dashboardName = dashboardName;
            this.key = rawData.split("<")[0];
            String type = rawData.split("<")[1].split(">")[0];
            this.stringifiedData = rawData.split(":")[1];
            switch (type.toLowerCase()) {
                case "string":
                    this.type = types.STRING;
                    break;
                case "double":
                    this.type = types.DOUBLE;
                    break;
                case "boolean":
                    this.type = types.BOOLEAN;
                    break;
                case "int":
                    this.type = types.INTEGER;
                    break;
                default:
                    throw new IllegalArgumentException("Unknown type: " + type);
            }
        }

        public String getString() {
            if (type != types.STRING) {
                throw new IllegalStateException("Value is not a string");
            }
            return stringifiedData;
        }

        public double getDouble() {
            if (type != types.DOUBLE) {
                throw new IllegalStateException("Value is not a double");
            }
            return Double.parseDouble(stringifiedData);
        }

        public boolean getBoolean() {
            if (type != types.BOOLEAN) {
                throw new IllegalStateException("Value is not a boolean");
            }
            return Boolean.parseBoolean(stringifiedData);
        }

        public int getInt() {
            if (type != types.INTEGER) {
                throw new IllegalStateException("Value is not an integer");
            }
            return Integer.parseInt(stringifiedData);
        }

        public String getKey() {
            return key;
        }

        public String getDashboardName() {
            return dashboardName;
        }

        public Class<?> getType() {
            switch (type) {
                case STRING:
                    return String.class;
                case DOUBLE:
                    return Double.class;
                case BOOLEAN:
                    return Boolean.class;
                case INTEGER:
                    return Integer.class;
                default:
                    throw new IllegalStateException("Unknown type: " + type);
            }
        }

        private enum types {
            STRING,
            DOUBLE,
            BOOLEAN,
            INTEGER
        }
    }
}
