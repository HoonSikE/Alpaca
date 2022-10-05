package com.example.taxi.data.dto.user.destination

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class Destination(
    val address: String = "",
    val latitude: String = "",
    val addressName: String = "",
    val longitude: String = ""
) : Parcelable