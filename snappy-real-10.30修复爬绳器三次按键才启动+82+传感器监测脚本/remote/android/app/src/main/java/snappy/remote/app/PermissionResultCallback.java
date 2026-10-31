/**
 * @file PermissionResultCallback.java
 * @brief 权限申请结果回调
 */

package snappy.remote.app;

public interface PermissionResultCallback {
    void onPermissionGranted();
    void onPermissionDenied(String[] deniedPermissions);
}