package com.example.taxi.utils.constant


const val USER_SEQ = "user_seq"
const val NAME = "name"
const val USECOUNT = "useCount"
const val PROFILEIMAGE = "profileImage"
const val TEL = "tel"
const val HOME = "home"
const val COMPANY = "company"
const val CARNUMBER = "carNumber"
const val RIDECOMFORTAVERAGE= "rideComfortAverage"
const val FEE = "Fee"
const val DISTANCE = "distance"
const val MILEAGE = "Mileage"
const val CARIMAGE = "carImage"
const val CARNAME = "carName"
const val LATITUDE = "latitude"
const val LONGITUDE = "longitude"
const val CLEANLINESSAVERAGE = "cleanlinessAverage"
const val ISEACHDRIVING = "isEachDriving"
const val ISEACHPROVIDER = "isEachProvider"
const val ISEACHINOPERATION = "isEachInOperation"
const val PROVIDERID = "providerId"
const val DESTINATIONLATITUDE = "destinationLatitude"
const val DESTINATIONLONGITUDE = "destinationLongitude"
const val DESTINATIONNAME = "destinationName"
const val DESTINATIONADDRESS = "destinationAddress"
const val STARTLATITUDE = "startLatitude"
const val STARTLONGITUDE = "startLongitude"
const val STARTNAME = "startName"

/**
 * 출발전 / 도착 후 상태
 */
const val START = 90
const val END = 91

object FireStoreCollection{
    val DESTINATION = "Destination"
    val LASTDESTINATION = "LastDestination"
    val FAVORITES = "Favorites"
    val USER = "User"
    val INSIDECARLIST = "InsideCarList"
    val USERADDRESSINFO = "UserAddressInfo"
    val ROUTE = "Route"
    val ROUTESETTING = "RouteSetting"
    val TAXILIST = "TaxiList"
    val DISTANCE = "Distance"
    val CURRENTLOCATION = "CurrentLocation"
    val USERLIST = "UserList"
    val PROVIDER = "Provider"
    val BOARDEDTAXILIST = "BoardedTaxiList"
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