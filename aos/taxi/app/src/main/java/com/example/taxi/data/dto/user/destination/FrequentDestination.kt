package com.example.taxi.data.dto.user.destination

import android.os.Parcelable
import com.google.firebase.firestore.ServerTimestamp
import com.google.gson.annotations.SerializedName
import kotlinx.parcelize.Parcelize
import java.util.*

@Parcelize
data class FrequentDestination(
    val address: String = "",
    val latitude: String = "",
    val count: Int = 0,
    val addressName: String = "",
    val longitude: String = ""
) : Parcelable
