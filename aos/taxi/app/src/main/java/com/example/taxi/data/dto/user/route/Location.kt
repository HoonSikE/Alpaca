package com.example.taxi.data.dto.user.route

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class Location(
    var lati: String = "",
    var long: String = "",
) : Parcelable
