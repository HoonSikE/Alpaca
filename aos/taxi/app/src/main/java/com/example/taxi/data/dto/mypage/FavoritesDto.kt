package com.example.taxi.data.dto.mypage

import android.os.Parcelable
import com.example.taxi.data.dto.user.destination.Destination
import kotlinx.parcelize.Parcelize

@Parcelize
data class FavoritesDto(
    var favorites: MutableList<Favorites> = mutableListOf()
) : Parcelable
