package com.example.taxi.data.dto.user.boarded_taxi_list

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class BoardedTaxi (
    var Cost: Int = 0,
    val Destination: String = "",
    val StartingPoint: String = "",
    val TimeTaken: Int = 0,
    val carImage : String = "",
    val carName: String = "",
    val carNumber: String = "",
    val cleanlinessAverage: Double = 0.0,
    val rideComfortAverage: Double = 0.0
) : Parcelable