package com.example.taxi.data.repository

import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.utils.constant.UiState

interface DestinationRepository {
    fun getDestination(result: (UiState<List<Destination>>) -> Unit)
    fun addDestination(destination: Destination, result: (UiState<Destination>) -> Unit)
    fun getFavorites(result: (UiState<List<Favorites>>) -> Unit)
    fun addFavorites(destination: Destination, result: (UiState<Favorites>) -> Unit)
}