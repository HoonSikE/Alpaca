package com.example.taxi.data.dto.user.address_info

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class AddressInfo(
    var company : String = "",
    var home : String = ""
) : Parcelable
