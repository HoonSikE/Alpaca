package com.example.taxi.data.dto.mypage

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class Favorites(
    val address: String = "",
    val latitude: String = "",
    val addressName: String = "",
    val longitude: String = ""
) : Parcelable
