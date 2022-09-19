package com.example.taxi.di

import android.app.Application
import android.content.Context
import com.example.taxi.utils.preference.PreferenceUtil
import dagger.hilt.android.HiltAndroidApp

@HiltAndroidApp
class ApplicationClass: Application() {
    companion object {
        lateinit var userId: String
        private lateinit var application: ApplicationClass
        fun getInstance() : ApplicationClass = application

        lateinit var prefs: PreferenceUtil
    }

    override fun onCreate(){
        prefs = PreferenceUtil(applicationContext)
        super.onCreate()
        application = this
    }

}