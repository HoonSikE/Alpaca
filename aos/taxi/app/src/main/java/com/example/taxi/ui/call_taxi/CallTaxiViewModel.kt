package com.example.taxi.ui.call_taxi

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.driving.CurrentLocation
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

    private val _taxiList = MutableLiveData<UiState<Taxi>>()
    val taxiList: LiveData<UiState<Taxi>>
        get() = _taxiList

    private val _taxiListUpdate = MutableLiveData<UiState<Taxi>>()
    val taxiListUpdate: LiveData<UiState<Taxi>>
        get() = _taxiListUpdate

    private val _distance = MutableLiveData<UiState<Int>>()
    val distance: LiveData<UiState<Int>>
        get() = _distance

    private val _currentLocation = MutableLiveData<UiState<CurrentLocation>>()
    val currentLocation: LiveData<UiState<CurrentLocation>>
        get() = _currentLocation

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

    fun getDistance() {
        _distance.value = UiState.Loading
        repository.getDistance { _distance.value = it }
    }

    fun updateTaxiList(taxiList : Taxi) {
        _taxiListUpdate.value = UiState.Loading
        repository.updateTaxiList(taxiList) { _taxiListUpdate.value = it }
    }

    fun getCurrentLocation(){
        _currentLocation.value = UiState.Loading
        repository.getCurrentLocation { _currentLocation.value = it }
    }

}