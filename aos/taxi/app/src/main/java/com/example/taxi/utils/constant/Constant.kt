package com.example.taxi.utils.constant


const val USER_SEQ = "user_seq"
const val NAME = "name"
const val USECOUNT = "useCount"
const val CARNUMBER = "carNumber"
const val RIDECOMFORTAVERAGE= "rideComfortAverage"
const val CARIMAGE = "carImage"
const val LATITUDE = "latitude"
const val LONGITUDE = "longitude"
const val CLEANLINESSAVERAGE = "cleanlinessAverage"
const val ISEACHDRIVING = "isEachDriving"
const val ISEACHINOPERATION = "isEachInOperation"

/**
 * 출발전 / 도착 후 상태
 */
const val START = 90
const val END = 91

object FireStoreCollection{
    val DESTINATION = "Destination"
    val FAVORITES = "Favorites"
    val USER = "User"
    val ROUTE = "Route"
    val ROUTESETTING = "RouteSetting"
    val TAXILIST = "TaxiList"
}

object FireStoreDocumentField {
    val DATE = "date"
    val USER_ID = "user_id"
}

object FirebaseStorageConstants {
    val ROOT_DIRECTORY = "app"
    val NOTE_IMAGES = "note"
}

object SharedPrefConstants {
    val LOCAL_SHARED_PREF = "local_shared_pref"
    val USER_SESSION = "user_session"
}

object KakaoApi {
    val BASE_URL = "https://dapi.kakao.com/"
    val API_KEY = "KakaoAK 71db95f3e177c75c9ef05e627c8d6c71"
}