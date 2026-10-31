package snappy.remote.app;

public interface PermissionResultCallback {
    void onPermissionGranted();
    void onPermissionDenied(String[] deniedPermissions);
}