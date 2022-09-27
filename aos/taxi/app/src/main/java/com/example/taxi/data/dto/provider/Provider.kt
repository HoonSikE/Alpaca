package com.example.taxi.data.dto.provider

data class Provider(
    var car: ProviderCar? = null,
    @field:JvmField
    val driverLicense: Boolean = false,
    val revenue : Int = 0
)
