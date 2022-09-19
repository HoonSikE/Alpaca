package com.example.taxi.utils.preference

import android.content.Context
import android.content.SharedPreferences
import android.provider.ContactsContract.DisplayNameSources.NICKNAME
import com.example.taxi.utils.constant.*

class PreferenceUtil(context: Context) {
    private val prefs: SharedPreferences = context.getSharedPreferences("taxi", Context.MODE_PRIVATE)

    var userSeq: String?
        get() = prefs.getString(NAME, null)
        set(value) {
            prefs.edit().putString(USER_SEQ, value).apply()
        }

    var name: String?
        get() = prefs.getString(NAME, null)
        set(value) {
            prefs.edit().putString(NAME, value).apply()
        }

}