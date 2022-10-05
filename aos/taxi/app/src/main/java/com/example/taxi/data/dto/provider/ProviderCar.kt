package com.example.taxi.data.dto.provider

import android.os.Parcelable
import com.example.taxi.data.dto.user.route.Location
import kotlinx.parcelize.Parcelize

@Parcelize
data class ProviderCar (
    var carImage : String = "",
    var carName : String = "",
    var carNumber : String = "",
    var cleanlinessAverage : Double = 0.0,
    var deadLine: String = "",
    @field:JvmField
    var isEachDriving : Boolean = false,
    @field:JvmField
    var isEachInOperation : Boolean = false,
    var position: Location = Location("",""),
    var rideComfortAverage : Double = 0.0,
) : Parcelable