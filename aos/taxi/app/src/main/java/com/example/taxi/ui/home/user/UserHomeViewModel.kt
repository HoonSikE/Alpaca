package com.example.taxi.ui.home.user

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.data.repository.FrequentDestinationRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class UserHomeViewModel @Inject constructor(
    private val repository: FrequentDestinationRepository
): ViewModel() {
    
    private val _destinations = MutableLiveData<UiState<List<FrequentDestination>>>()
    val destinations: LiveData<UiState<List<FrequentDestination>>>
        get() = _destinations

    private val _updateDestinations = MutableLiveData<UiState<List<FrequentDestination>>>()
    val updateDestinations: LiveData<UiState<List<FrequentDestination>>>
        get() = _updateDestinations

    private val _lastDestinations = MutableLiveData<UiState<List<Destination>>>()
    val lastDestinations: LiveData<UiState<List<Destination>>>
        get() = _lastDestinations

    private val _updateLastDestinations = MutableLiveData<UiState<List<Destination>>>()
    val updateLastDestinations: LiveData<UiState<List<Destination>>>
        get() = _updateLastDestinations

    private val _favorites = MutableLiveData<UiState<List<Favorites>>>()
    val favorites: LiveData<UiState<List<Favorites>>>
        get() = _favorites

    private val _updateFavorites = MutableLiveData<UiState<String>>()
    val updateFavorites: LiveData<UiState<String>>
        get() = _updateFavorites

    private val _deleteFavorites = MutableLiveData<UiState<String>>()
    val deleteFavorites: LiveData<UiState<String>>
        get() = _deleteFavorites

    fun getDestinations() {
        _destinations.value = UiState.Loading
        repository.getDestination { _destinations.value = it }
    }

    fun updateDestinations(destination: List<FrequentDestination>) {
        _updateDestinations.value = UiState.Loading
        repository.updateDestination(destination) { _updateDestinations.value = it }
    }

    fun getLastDestinations() {
        _lastDestinations.value = UiState.Loading
        repository.getLastDestination { _lastDestinations.value = it }
    }

    fun updateLastDestinations(destination: List<Destination>) {
        _updateLastDestinations.value = UiState.Loading
        repository.updateLastDestination(destination) { _updateLastDestinations.value = it }
    }

    fun getFavorites() {
        _favorites.value = UiState.Loading
        repository.getFavorites { _favorites.value = it }
    }

    fun updateFavorites(favorites: List<Favorites>) {
        _updateFavorites.value = UiState.Loading
        repository.updateFavorites(favorites) { _updateFavorites.value = it }
    }

    fun deleteFavorites() {
        _deleteFavorites.value = UiState.Loading
        repository.deleteFavorites { _deleteFavorites.value = it }
    }
}