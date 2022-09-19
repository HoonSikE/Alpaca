package com.ssafy.daero.utils.view

import android.app.Activity
import android.app.Dialog
import android.content.Context
import android.os.Build
import android.os.Bundle
import android.text.Editable
import android.util.DisplayMetrics
import android.view.View
import android.view.ViewGroup
import android.view.WindowInsets
import android.view.WindowManager
import android.view.inputmethod.InputMethodManager
import android.widget.EditText
import android.widget.ImageView
import androidx.core.content.ContentProviderCompat
import androidx.core.content.ContentProviderCompat.requireContext
import androidx.core.content.ContextCompat
import androidx.core.graphics.drawable.DrawableCompat
import androidx.core.view.WindowCompat
import androidx.core.widget.doAfterTextChanged
import androidx.fragment.app.Fragment
import androidx.recyclerview.widget.LinearSmoothScroller
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetDialog
import com.google.android.material.bottomsheet.BottomSheetDialogFragment
import com.google.android.material.snackbar.Snackbar

// Milliseconds used for UI animations in Camera
const val ANIMATION_FAST_MILLIS = 50L
const val ANIMATION_SLOW_MILLIS = 100L

fun View.setStatusBarTransparent(activity: Activity) = activity.run {
    WindowCompat.setDecorFitsSystemWindows(window, false)
    setPaddingWhenStatusBarTransparent(this)
}

fun Activity.setStatusBarOrigin() {
    WindowCompat.setDecorFitsSystemWindows(window, true)
}

fun View.setPaddingWhenStatusBarTransparent(context: Context) = context.run {
    setPadding(0, this.statusBarHeight(), 0, this.navigationHeight())
}

fun View.setPaddingBottomWhenStatusBarTransparent(context: Context) = context.run {
    setPadding(0, 0, 0, this.navigationHeight())
}

fun View.setPaddingTopWhenStatusBarTransparent(context: Context) = context.run {
    setPadding(0, this.statusBarHeight(), 0, 0)
}

fun Fragment.setStatusBarBackground() {
    requireActivity().window.statusBarColor = ContextCompat.getColor(requireContext(), R.color.background)
}

fun Fragment.setStatusBarOriginTransparent() {
    requireActivity().window.statusBarColor = ContextCompat.getColor(requireContext(), R.color.transparent)
}

fun Activity.hideKeyboard(editText: EditText) {
    val imm = getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager
    imm.hideSoftInputFromWindow(editText.windowToken, 0)
}

fun ImageView.setTint(colorId: Int) {
    DrawableCompat.setTint(DrawableCompat.wrap(drawable), colorId)
}

fun Context.getSmoothScroll(): LinearSmoothScroller {
    return object : LinearSmoothScroller(this) {
        override fun getVerticalSnapPreference(): Int {
            return SNAP_TO_START
        }
    }
}

fun getScreenHeight(context: Context): Int {
    val wm = context.getSystemService(Context.WINDOW_SERVICE) as WindowManager
    return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
        val windowMetrics = wm.currentWindowMetrics
        val insets = windowMetrics.windowInsets
            .getInsetsIgnoringVisibility(WindowInsets.Type.systemBars())
        windowMetrics.bounds.height() - insets.bottom - insets.top
    } else {
        val displayMetrics = DisplayMetrics()
        wm.defaultDisplay.getMetrics(displayMetrics)
        displayMetrics.heightPixels
    }
}

fun RecyclerView.scrollToPositionSmooth(int: Int) {
    this.layoutManager?.startSmoothScroll(this.context.getSmoothScroll().apply {
        targetPosition = int
    })
}

fun RecyclerView.scrollToPositionFast(position: Int) {
    this.layoutManager?.scrollToPosition(position)
}

fun EditText.isPosOutOf(x: Float, y: Float) = x < left || x > right || y < top || y > bottom

fun BottomSheetDialogFragment.expandFullHeight() {
    val bottomSheet = dialog?.findViewById(com.google.android.material.R.id.design_bottom_sheet) as View
    val behavior = BottomSheetBehavior.from<View>(bottomSheet)
    val layoutParams = bottomSheet.layoutParams
    layoutParams.height = ViewGroup.LayoutParams.MATCH_PARENT
    bottomSheet.layoutParams = layoutParams
    behavior.state = BottomSheetBehavior.STATE_EXPANDED
    behavior.skipCollapsed = true
}

fun showSnackbar(view: View, message: String){
    Snackbar.make(view,message, Snackbar.LENGTH_INDEFINITE)
        .setAction("확인"){}
        .show()
}

fun BottomSheetDialogFragment.setFullHeight() : BottomSheetDialog {
    return BottomSheetDialog(requireContext(), theme).apply {
        setOnShowListener { bottomSheetDialog ->
            (bottomSheetDialog as BottomSheetDialog).findViewById<View>(com.google.android.material.R.id.design_bottom_sheet)?.let { view ->
                BottomSheetBehavior.from(view).apply {
                    state = BottomSheetBehavior.STATE_EXPANDED
                    isDraggable = false
                }

                val layoutParams = view.layoutParams
                layoutParams.height = WindowManager.LayoutParams.MATCH_PARENT
                view.layoutParams = layoutParams
            }
        }
    }
}