package com.example.taxi.data.dto.provider

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class UserList(
    var user: List<TaxiUser> = listOf()
) : Parcelable
