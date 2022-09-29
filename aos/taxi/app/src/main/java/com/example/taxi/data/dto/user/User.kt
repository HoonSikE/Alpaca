package com.example.taxi.data.dto.user

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class User(
    val name: String = "",
    var profileImage: String = "",
    val tel: String = "",
    val useCount: Int = 0,
    val userId: String = "",
    var userSeq: String = "",
    @field:JvmField
    val isEachProvider: Boolean = false,
) : Parcelable
