package com.example.taxi.ui.call_taxi

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
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
    private val _route = MutableLiveData<UiState<List<Location>>>()
    val route: LiveData<UiState<List<Location>>>
        get() = _route

    private val _routeSetting = MutableLiveData<UiState<RouteSetting>>()
    val routeSetting: LiveData<UiState<RouteSetting>>
        get() = _routeSetting

    fun getRoute() {
        _route.value = UiState.Loading
        repository.getRoute { _route.value = it }
    }

    fun addRouteSetting(routeSetting: RouteSetting){
        _routeSetting.value = UiState.Loading
        repository.addRouteSetting(routeSetting) { _routeSetting.value = it }
    }
}