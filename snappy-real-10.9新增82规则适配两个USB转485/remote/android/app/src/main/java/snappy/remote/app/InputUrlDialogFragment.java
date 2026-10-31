/**
 * @file InputUrlDialogFragment.java
 * @brief 手动输入视频地址的对话框
 *
 * @details
 * - 回显当前地址，确认后交由调用方处理。
 * - 只做输入与校验，不直接保存配置。
 */

package snappy.remote.app;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.text.TextUtils;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.EditText;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.DialogFragment;

public class InputUrlDialogFragment extends DialogFragment {
    private static final String KEY_POSITIVE = "positive";
    private static final String KEY_NEGATIVE = "negative";
    private static final String KEY_URL = "url";

    private ButtonOnClickListener positiveListener;
    private ButtonOnClickListener negativeListener;

    @NonNull
    @Override
    public Dialog onCreateDialog(@Nullable Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(requireActivity());
        LayoutInflater inflater = requireActivity().getLayoutInflater();
        View view = inflater.inflate(R.layout.dialog_input_url, null);

        EditText editText = view.findViewById(R.id.et_url);
        editText.setText(getStringArgument(KEY_URL));

        String positive = getStringArgument(KEY_POSITIVE);
        if (TextUtils.isEmpty(positive)) {
            positive = "确定";
        }

        String negative = getStringArgument(KEY_NEGATIVE);
        if (TextUtils.isEmpty(negative)) {
            negative = "取消";
        }

        builder.setView(view)
                .setPositiveButton(positive, (dialog, which) -> {
                    ButtonOnClickListener listener = positiveListener;
                    if (listener != null) {
                        listener.onClick(
                                dialog,
                                which,
                                editText.getText().toString().trim()
                        );
                    }
                })
                .setNegativeButton(negative, (dialog, which) -> {
                    ButtonOnClickListener listener = negativeListener;
                    if (listener != null) {
                        listener.onClick(dialog, which, "");
                    }
                });

        return builder.create();
    }

    private String getStringArgument(String key) {
        Bundle args = getArguments();
        if (args == null) {
            return "";
        }
        String value = args.getString(key);
        return value == null ? "" : value;
    }

    public static InputUrlDialogFragment createDialog(
            String positive,
            ButtonOnClickListener positiveListener,
            String negative,
            ButtonOnClickListener negativeListener,
            String defaultUrl
    ) {
        InputUrlDialogFragment dialog = new InputUrlDialogFragment();
        Bundle args = new Bundle();

        if (!TextUtils.isEmpty(positive)) {
            args.putString(KEY_POSITIVE, positive);
        }
        if (!TextUtils.isEmpty(negative)) {
            args.putString(KEY_NEGATIVE, negative);
        }
        if (!TextUtils.isEmpty(defaultUrl)) {
            args.putString(KEY_URL, defaultUrl);
        }

        dialog.setArguments(args);
        dialog.positiveListener = positiveListener;
        dialog.negativeListener = negativeListener;
        return dialog;
    }

    public interface ButtonOnClickListener {
        void onClick(DialogInterface dialog, int id, String value);
    }
}