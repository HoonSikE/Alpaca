package com.example.taxi.data.dto.user.destination

import android.os.Parcelable
import com.google.gson.annotations.SerializedName
import kotlinx.parcelize.Parcelize

@Parcelize
data class DestinationDto(
    var destination: MutableList<Destination> = mutableListOf()
) : Parcelable
