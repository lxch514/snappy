package snappy.remote.g20;

import com.skydroid.rcsdk.KeyManager;
import com.skydroid.rcsdk.common.DeviceType;
import com.skydroid.rcsdk.common.callback.CompletionCallbackWith;
import com.skydroid.rcsdk.common.callback.KeyListener;
import com.skydroid.rcsdk.common.error.SkyException;
import com.skydroid.rcsdk.key.RemoteControllerKey;

public class ReadRCButtonHelper {
    private static final long INTERVAL_MS = 99;

    private long lastH16CallbackMs = 0;
    private RCButtonValueListener listener;
    private LopperThread lopperThread;
    private final DeviceType deviceType;
    private boolean started = false;

    public ReadRCButtonHelper(DeviceType deviceType) {
        this.deviceType = deviceType;
    }

    private final KeyListener<int[]> channelKeyListener = new KeyListener<int[]>() {
        @Override
        public void onValueChange(int[] oldValue, int[] newValue) {
            long nowMs = System.currentTimeMillis();
            if (nowMs - lastH16CallbackMs >= INTERVAL_MS) {
                RCButtonValueListener current = listener;
                if (current != null) {
                    current.onRCButtonValue(newValue);
                }
                lastH16CallbackMs = nowMs;
            }
        }
    };

    private final CompletionCallbackWith<int[]> channelKeyCallback = new CompletionCallbackWith<int[]>() {
        @Override
        public void onSuccess(int[] values) {
            RCButtonValueListener current = listener;
            if (current != null) {
                current.onRCButtonValue(values);
            }
        }

        @Override
        public void onFailure(SkyException e) {
        }
    };

    public void setListener(RCButtonValueListener listener) {
        this.listener = listener;
    }

    public synchronized void start() {
        if (started) {
            return;
        }

        if (deviceType == DeviceType.H16) {
            KeyManager.INSTANCE.listen(
                    RemoteControllerKey.INSTANCE.getKeyH16Channels(),
                    channelKeyListener
            );
        } else {
            lopperThread = new LopperThread();
            lopperThread.start();
        }

        started = true;
    }

    public synchronized void stop() {
        if (!started) {
            return;
        }

        started = false;
        KeyManager.INSTANCE.cancelListen(channelKeyListener);

        if (lopperThread != null) {
            lopperThread.close();
            lopperThread = null;
        }
    }

    public interface RCButtonValueListener {
        void onRCButtonValue(int[] buttons);
    }

    private class LopperThread extends Thread {
        private boolean running = false;

        @Override
        public void run() {
            while (running) {
                KeyManager.INSTANCE.get(
                        RemoteControllerKey.INSTANCE.getKeyChannels(),
                        channelKeyCallback
                );

                synchronized (this) {
                    try {
                        wait(INTERVAL_MS);
                    } catch (InterruptedException e) {
                        interrupt();
                    }
                }
            }
        }

        @Override
        public synchronized void start() {
            running = true;
            super.start();
        }

        public void close() {
            synchronized (this) {
                running = false;
                try {
                    notify();
                } catch (Exception ignored) {
                }
            }

            try {
                join();
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }
}