package com.example.taxi.data.dto.mypage

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class Chat(
    val userId : String = "",
    val userName : String = "",
    val profileImageUrl : String = "",
) : Parcelable
