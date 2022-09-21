package com.example.taxi.data.dto.user.route

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class Route(
    var position : MutableList<Location> = mutableListOf()
) : Parcelable
