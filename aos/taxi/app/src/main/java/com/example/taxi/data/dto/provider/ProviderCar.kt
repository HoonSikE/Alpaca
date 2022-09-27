package com.example.taxi.data.dto.provider

import com.example.taxi.data.dto.user.route.Location

data class ProviderCar (
    var carNumber : String = "",
    var rideComfortAverage : Double = 0.0,
    var carImage : String = "",
    var position: Location = Location("",""),
    var cleanlinessAverage : Double = 0.0,
    var deadLine: String = "",
    @field:JvmField
    var isEachDriving : Boolean = false,
    @field:JvmField
    var isEachInOperation : Boolean = false,
)