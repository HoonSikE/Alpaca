package com.example.taxi.data.dto.user.calltaxi

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class TaxiList(
    var taxiList: List<Taxi> = listOf()
) : Parcelable
