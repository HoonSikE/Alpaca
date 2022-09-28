package com.example.taxi.data.repository

import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.utils.constant.UiState

interface ProviderRepository {
    fun getProvider(result: (UiState<Provider>) -> Unit)
    fun updateProvider(provider: ProviderCar, result: (UiState<ProviderCar>) -> Unit)
    fun updateRevenue(revenue: Int, result: (UiState<Int>) -> Unit)
    fun getUserList (result: (UiState<UserList>) -> Unit)
    fun updateUserList(userList: UserList, result: (UiState<UserList>) -> Unit)
}