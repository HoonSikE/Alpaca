package com.example.taxi.data.dto.user.destination

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class FrequentDestinationDto(
    var frequentDestination: MutableList<FrequentDestination> = mutableListOf()
) : Parcelable
