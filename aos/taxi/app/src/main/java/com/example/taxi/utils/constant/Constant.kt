package com.example.taxi.utils.constant


const val USER_SEQ = "user_seq"
const val NAME = "name"

/**
 * 출발전 / 도착 후 상태
 */
const val START = 90
const val END = 91

object FireStoreCollection{
    val DESTINATION = "Destination"
    val FAVORITES = "Favorites"
    val USER = "User"
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