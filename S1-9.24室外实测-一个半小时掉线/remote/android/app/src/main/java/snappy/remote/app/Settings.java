package snappy.remote.app;

import android.content.Context;
import android.content.SharedPreferences;

import snappy.remote.g20.G20RemoteDefaults;

public class Settings {
    public static final String DEFAULT_VIDEO_URL = G20RemoteDefaults.DEFAULT_VIDEO_URL;

    private static final String PREFS = "snappy_remote_settings";
    private static final String KEY_LAST_URL = "last_url";
    private static final String KEY_ROBOT_ID = "robot_id";

    private final SharedPreferences preferences;

    public Settings(Context context) {
        preferences = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    public String getLastUrl() {
        return preferences.getString(KEY_LAST_URL, DEFAULT_VIDEO_URL);
    }

    public void setLastUrl(String url) {
        preferences.edit().putString(KEY_LAST_URL, url).apply();
    }

    public String getRobotId() {
        return preferences.getString(KEY_ROBOT_ID, G20RemoteDefaults.ROBOT_ID);
    }

    public void setRobotId(String robotId) {
        preferences.edit().putString(KEY_ROBOT_ID, robotId).apply();
    }
}