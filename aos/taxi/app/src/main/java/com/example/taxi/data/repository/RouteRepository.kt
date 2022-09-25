package com.example.taxi.data.repository

import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.utils.constant.UiState

interface RouteRepository {
    fun getRoute(result: (UiState<List<String>>) -> Unit)
    fun addRouteSetting(routeSetting: RouteSetting, result: (UiState<RouteSetting>) -> Unit)
    fun getTaxiList(result: (UiState<List<Taxi>>) -> Unit)
}