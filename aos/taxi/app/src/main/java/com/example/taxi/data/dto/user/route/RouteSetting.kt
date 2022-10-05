package com.example.taxi.data.dto.user.route

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class RouteSetting(
    var destination : Location = Location("",""),
    var startingPoint : Location = Location("",""),
    var checkState : Boolean = false
) : Parcelable
