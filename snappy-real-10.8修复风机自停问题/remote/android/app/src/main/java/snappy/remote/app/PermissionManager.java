/**
 * @file PermissionManager.java
 * @brief 运行时权限申请
 *
 * @details
 * - 统一申请网络、存储等运行所需权限。
 * - 结果通过回调返回，界面据此决定后续流程。
 */

package snappy.remote.app;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.os.Build;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.List;

public class PermissionManager {
    public static final int REQUEST_CODE = 3101;

    private PermissionResultCallback callback;

    public void requestRuntimePermissions(Activity activity, PermissionResultCallback callback) {
        this.callback = callback;

        List<String> permissions = new ArrayList<>();
        permissions.add(Manifest.permission.INTERNET);
        permissions.add(Manifest.permission.ACCESS_WIFI_STATE);
        permissions.add(Manifest.permission.CHANGE_WIFI_MULTICAST_STATE);

        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            permissions.add(Manifest.permission.READ_EXTERNAL_STORAGE);
            permissions.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        }

        List<String> missing = new ArrayList<>();
        for (String permission : permissions) {
            if (ContextCompat.checkSelfPermission(activity, permission) != PackageManager.PERMISSION_GRANTED) {
                missing.add(permission);
            }
        }

        if (missing.isEmpty()) {
            if (callback != null) {
                callback.onPermissionGranted();
            }
            return;
        }

        ActivityCompat.requestPermissions(activity, missing.toArray(new String[0]), REQUEST_CODE);
    }

    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode != REQUEST_CODE) {
            return;
        }

        List<String> denied = new ArrayList<>();
        for (int i = 0; i < permissions.length; ++i) {
            if (i >= grantResults.length || grantResults[i] != PackageManager.PERMISSION_GRANTED) {
                denied.add(permissions[i]);
            }
        }

        PermissionResultCallback current = callback;
        if (current == null) {
            return;
        }

        if (denied.isEmpty()) {
            current.onPermissionGranted();
        } else {
            current.onPermissionDenied(denied.toArray(new String[0]));
        }
    }
}