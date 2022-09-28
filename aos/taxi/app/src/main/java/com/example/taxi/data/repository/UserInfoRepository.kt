package com.example.taxi.data.repository

import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.utils.constant.UiState

interface UserInfoRepository {
    fun getAddressInfo(result: (UiState<AddressInfo>) -> Unit)
    fun addAddressInfo(addressInfo: AddressInfo, result: (UiState<AddressInfo>) -> Unit)
    fun addImageUpLoad(user: User, result: (UiState<User>) -> Unit)
    fun updateUserTel(tel: String, result: (UiState<String>) -> Unit)
    fun deleteUserAddress(result: (UiState<String>) -> Unit)
    fun deleteImage(result: (UiState<String>) -> Unit)
    fun updateBoardedTaxiList(
        boardedTaxi: List<BoardedTaxi>,
        result: (UiState<List<BoardedTaxi>>) -> Unit
    )

    fun getBoardedTaxiList(result: (UiState<BoardedTaxiList>) -> Unit)
}