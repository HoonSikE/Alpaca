package com.example.taxi.data.dto.user.boarded_taxi_list

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class BoardedTaxi(
    var userId: String = "",
    var cost: Int = 0,
    val destination: String = "",
    val startingPoint: String = "",
    val carImage: String = "",
    val carName: String = "",
    val carNumber: String = "",
    var cleanlinessAverage: Double = 0.0,
    var rideComfortAverage: Double = 0.0,
    val distance: Double = 0.0
) : Parcelable