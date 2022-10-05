package com.example.taxi.data.dto.common

data class PhotoList(
    var carNumber: String = "",
    var start : MutableList<String> = mutableListOf(),
    var end : MutableList<String> = mutableListOf()
)
