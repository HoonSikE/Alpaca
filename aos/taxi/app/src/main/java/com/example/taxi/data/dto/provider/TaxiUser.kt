package com.example.taxi.data.dto.provider

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class TaxiUser(
    var userId : String = "",
    var userName : String = "",
    var rideComfort : Double = 0.0,
    var userImage : String = "",
    var cleanliness : Double = 0.0,
) : Parcelable