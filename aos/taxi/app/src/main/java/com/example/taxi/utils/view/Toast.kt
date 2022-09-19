package com.example.taxi.utils.view

import android.app.Activity
import android.graphics.Color
import android.widget.Toast
import androidx.fragment.app.Fragment
import com.example.taxi.R
import io.github.muddz.styleabletoast.StyleableToast

fun Fragment.toast(text: String) {
    StyleableToast
        .Builder(requireContext())
        .text(text)
        .textColor(R.color.primaryTextColor)
        .backgroundColor(Color.WHITE)
        .stroke(1, R.color.primaryColor)
        .length(Toast.LENGTH_SHORT)
        .textBold()
        .cornerRadius(8)
        .show()
}

fun Activity.toast(text: String) {
    StyleableToast
        .Builder(this)
        .text(text)
        .textColor(R.color.primaryTextColor)
        .backgroundColor(Color.WHITE)
        .stroke(1, R.color.primaryColor)
        .length(Toast.LENGTH_SHORT)
        .textBold()
        .cornerRadius(8)
        .show()
}