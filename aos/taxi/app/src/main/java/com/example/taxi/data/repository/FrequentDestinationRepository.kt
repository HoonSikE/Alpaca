package com.example.taxi.data.repository

import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.utils.constant.UiState

interface FrequentDestinationRepository {
    fun getDestination(result: (UiState<List<FrequentDestination>>) -> Unit)
    fun updateDestination(provider: List<FrequentDestination>, result: (UiState<List<FrequentDestination>>) -> Unit)
    fun addDestination(frequentDestination: FrequentDestination, result: (UiState<FrequentDestination>) -> Unit)
    fun getFavorites(result: (UiState<List<Favorites>>) -> Unit)
    fun addFavorites(frequentDestination: FrequentDestination, result: (UiState<Favorites>) -> Unit)
    fun getLastDestination(result: (UiState<List<Destination>>) -> Unit)
    fun updateLastDestination(
        provider: List<Destination>,
        result: (UiState<List<Destination>>) -> Unit
    )
}