package com.example.taxi.data.dto.provider

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class Provider(
    var car: ProviderCar? = null,
    @field:JvmField
    val driverLicense: Boolean = false,
    val revenue : Int = 0
) : Parcelable
