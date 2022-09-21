package com.example.taxi.data.dto.mypage

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class FavoritesDto(
    var favorites: MutableList<Favorites> = mutableListOf()
) : Parcelable
