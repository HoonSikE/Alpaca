package com.example.taxi.data.repository

import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.driving.CurrentLocation
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.utils.constant.UiState

interface RouteRepository {
    fun getRoute(result: (UiState<List<String>>) -> Unit)
    fun getDistance(result: (UiState<Int>) -> Unit)
    fun addRouteSetting(routeSetting: RouteSetting, result: (UiState<RouteSetting>) -> Unit)
    fun getTaxiList(result: (UiState<Taxi>) -> Unit)
    fun updateTaxiList(taxiList: Taxi, result: (UiState<Taxi>) -> Unit)
    fun getCurrentLocation(result: (UiState<CurrentLocation>) -> Unit)

}