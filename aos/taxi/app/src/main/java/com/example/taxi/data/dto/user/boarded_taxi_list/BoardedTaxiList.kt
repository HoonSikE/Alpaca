package com.example.taxi.data.dto.user.boarded_taxi_list

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class BoardedTaxiList(
    val taxiList: List<BoardedTaxi> = listOf()
) : Parcelable
