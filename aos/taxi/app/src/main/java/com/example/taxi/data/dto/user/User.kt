package com.example.taxi.data.dto.user

import android.os.Parcelable
import com.example.taxi.data.dto.user.destination.Destination
import com.google.gson.annotations.SerializedName
import kotlinx.parcelize.Parcelize
import kotlinx.parcelize.RawValue

@Parcelize
data class User(
    val name: String = "",
    val profileImage: String = "",
    val tel: String = "",
    val userId: String = "",
    var userSeq: String = "",
    val isEachProvider: Boolean = false,
) : Parcelable
