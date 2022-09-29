package com.example.taxi.ui.home.provider

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.repository.ProviderRepository
import com.example.taxi.data.repository.RouteRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class ProviderViewModel @Inject constructor(
    private val repository: ProviderRepository
): ViewModel() {
    private val _provider = MutableLiveData<UiState<Provider>>()
    val provider: LiveData<UiState<Provider>>
        get() = _provider

    private val _providerCar = MutableLiveData<UiState<ProviderCar>>()
    val providerCar: LiveData<UiState<ProviderCar>>
        get() = _providerCar

    private val _revenue = MutableLiveData<UiState<Int>>()
    val revenue: LiveData<UiState<Int>>
        get() = _revenue

    private val _userList = MutableLiveData<UiState<UserList>>()
    val userList: LiveData<UiState<UserList>>
        get() = _userList

    private val _upadteUserList = MutableLiveData<UiState<UserList>>()
    val upadteUserList: LiveData<UiState<UserList>>
        get() = _upadteUserList

    fun getProvider(){
        _provider.value = UiState.Loading
        repository.getProvider { _provider.value = it }
    }

    fun addProvider(provider: Provider) {
        _provider.value = UiState.Loading
        repository.addProvider(provider) { _provider.value = it }
    }

    fun updateProvider(providerCar: ProviderCar) {
        _providerCar.value = UiState.Loading
        repository.updateProvider(providerCar) { _providerCar.value = it }
    }

    fun updateRevenue(revenue : Int){
        _revenue.value = UiState.Loading
        repository.updateRevenue(revenue) { _revenue.value = it }
    }

    fun getUserList() {
        _userList.value = UiState.Loading
        repository.getUserList { _userList.value = it }
    }

    fun updateUserList(revenue : UserList){
        _upadteUserList.value = UiState.Loading
        repository.updateUserList(revenue) { _upadteUserList.value = it }
    }

}