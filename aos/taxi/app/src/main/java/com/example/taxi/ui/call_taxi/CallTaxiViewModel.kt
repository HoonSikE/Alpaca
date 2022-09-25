package com.example.taxi.ui.call_taxi

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.data.repository.RouteRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class CallTaxiViewModel @Inject constructor(
    private val repository: RouteRepository
): ViewModel() {
    private val _route = MutableLiveData<UiState<List<String>>>()
    val route: LiveData<UiState<List<String>>>
        get() = _route

    private val _routeSetting = MutableLiveData<UiState<RouteSetting>>()
    val routeSetting: LiveData<UiState<RouteSetting>>
        get() = _routeSetting

    private val _taxiList = MutableLiveData<UiState<List<Taxi>>>()
    val taxiList: LiveData<UiState<List<Taxi>>>
        get() = _taxiList

    fun getRoute() {
        _route.value = UiState.Loading
        repository.getRoute { _route.value = it }
    }

    fun addRouteSetting(routeSetting: RouteSetting){
        _routeSetting.value = UiState.Loading
        repository.addRouteSetting(routeSetting) { _routeSetting.value = it }
    }

    fun getTaxiList() {
        _taxiList.value = UiState.Loading
        repository.getTaxiList { _taxiList.value = it }
    }


}