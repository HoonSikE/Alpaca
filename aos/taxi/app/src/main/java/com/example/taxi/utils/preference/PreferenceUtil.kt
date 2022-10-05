package com.example.taxi.utils.preference

import android.content.Context
import android.content.SharedPreferences
import android.provider.ContactsContract.DisplayNameSources.NICKNAME
import com.example.taxi.utils.constant.*

class PreferenceUtil(context: Context) {
    private val prefs: SharedPreferences = context.getSharedPreferences("user", Context.MODE_PRIVATE)
    private val taxi: SharedPreferences = context.getSharedPreferences("taxi", Context.MODE_PRIVATE)
    private val destination: SharedPreferences = context.getSharedPreferences("destination", Context.MODE_PRIVATE)

    var destinationLatitude : String?
        get() = destination.getString(DESTINATIONLATITUDE, null)
        set(value){
            destination.edit().putString(DESTINATIONLATITUDE, value).apply()
        }

    var destinationLongitude : String?
        get() = destination.getString(DESTINATIONLONGITUDE, null)
        set(value){
            destination.edit().putString(DESTINATIONLONGITUDE, value).apply()
        }

    var destinationName : String?
        get() = destination.getString(DESTINATIONNAME, null)
        set(value){
            destination.edit().putString(DESTINATIONNAME, value).apply()
        }

    var destinationAddress : String?
        get() = destination.getString(DESTINATIONADDRESS, null)
        set(value){
            destination.edit().putString(DESTINATIONADDRESS, value).apply()
        }

    var startLatitude : String?
        get() = destination.getString(STARTLATITUDE, null)
        set(value){
            destination.edit().putString(STARTLATITUDE, value).apply()
        }

    var startLongitude : String?
        get() = destination.getString(STARTLONGITUDE, null)
        set(value){
            destination.edit().putString(STARTLONGITUDE, value).apply()
        }

    var startName : String?
        get() = destination.getString(STARTNAME, null)
        set(value){
            destination.edit().putString(STARTNAME, value).apply()
        }

    var carNumber : String?
        get() = taxi.getString(CARNUMBER, null)
        set(value){
            taxi.edit().putString(CARNUMBER, value).apply()
        }

    var rideComfortAverage : Float?
        get() = taxi.getFloat(RIDECOMFORTAVERAGE, 0.0F)
        set(value){
            if (value != null) {
                taxi.edit().putFloat(RIDECOMFORTAVERAGE, value).apply()
            }
        }

    var fee : Int?
        get() = taxi.getInt(FEE, 0)
        set(value){
            if (value != null) {
                taxi.edit().putInt(FEE, value).apply()
            }
        }

    var distance : Float?
        get() = taxi.getFloat(DISTANCE, 0.0F)
        set(value){
            if (value != null) {
                taxi.edit().putFloat(DISTANCE, value).apply()
            }
        }

    var carImage : String?
        get() = taxi.getString(CARIMAGE, null)
        set(value){
            taxi.edit().putString(CARIMAGE, value).apply()
        }

    var carName : String?
        get() = taxi.getString(CARNAME, null)
        set(value){
            taxi.edit().putString(CARNAME, value).apply()
        }

    var latitude : String?
        get() = taxi.getString(LATITUDE, null)
        set(value){
            taxi.edit().putString(LATITUDE, value).apply()
        }

    var longitude : String?
        get() = taxi.getString(LONGITUDE, null)
        set(value){
            taxi.edit().putString(LONGITUDE, value).apply()
        }

    var cleanlinessAverage : Float?
        get() = taxi.getFloat(CLEANLINESSAVERAGE, 0.0F)
        set(value){
            if (value != null) {
                taxi.edit().putFloat(CLEANLINESSAVERAGE, value).apply()
            }
        }

    var isEachDriving : Boolean?
        get() = taxi.getBoolean(ISEACHDRIVING, false)
        set(value){
            if (value != null) {
                taxi.edit().putBoolean(ISEACHDRIVING, value).apply()
            }
        }

    var isEachInOperation : Boolean?
        get() = taxi.getBoolean(ISEACHINOPERATION, false)
        set(value){
            if (value != null) {
                taxi.edit().putBoolean(ISEACHINOPERATION, value).apply()
            }
        }

    var providerId : String?
        get() = taxi.getString(PROVIDERID, null)
        set(value){
            taxi.edit().putString(PROVIDERID, value).apply()
        }

    var name: String?
        get() = prefs.getString(NAME, null)
        set(value) {
            prefs.edit().putString(NAME, value).apply()
        }

    var profileImage: String?
        get() = prefs.getString(PROFILEIMAGE, null)
        set(value) {
            if (value != null) {
                prefs.edit().putString(PROFILEIMAGE, value).apply()
            }
        }

    var tel: String?
        get() = prefs.getString(TEL, null)
        set(value) {
            prefs.edit().putString(TEL, value).apply()
        }

    var useCount: Int?
        get() = prefs.getInt(USECOUNT, 0)
        set(value) {
            if (value != null) {
                prefs.edit().putInt(USECOUNT, value).apply()
            }
        }

    var userSeq: String?
        get() = prefs.getString(USER_SEQ, null)
        set(value) {
            prefs.edit().putString(USER_SEQ, value).apply()
        }

    var isEachProvider : Boolean?
        get() = prefs.getBoolean(ISEACHPROVIDER, false)
        set(value){
            if (value != null) {
                prefs.edit().putBoolean(ISEACHPROVIDER, value).apply()
            }
        }

    var destinationUserName: String?
        get() = prefs.getString(DESTNATIONUSERNAME, null)
        set(value) {
            prefs.edit().putString(DESTNATIONUSERNAME, value).apply()
        }

    var destinationUserImg: String?
        get() = prefs.getString(DESTINATIONUSERIMG, null)
        set(value) {
            prefs.edit().putString(DESTINATIONUSERIMG, value).apply()
        }
}