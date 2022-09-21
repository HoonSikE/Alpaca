package com.example.taxi.ui.home.user

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.mypage.Favorites
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

    private val _addNote = MutableLiveData<UiState<Pair<FrequentDestination,String>>>()
    val addNote: LiveData<UiState<Pair<FrequentDestination,String>>>
        get() = _addNote

    private val _favorites = MutableLiveData<UiState<List<Favorites>>>()
    val favorites: LiveData<UiState<List<Favorites>>>
        get() = _favorites

    fun getDestinations() {
        _destinations.value = UiState.Loading
        repository.getDestination { _destinations.value = it }
    }

    fun getFavorites() {
        _favorites.value = UiState.Loading
        repository.getFavorites { _favorites.value = it }
    }
}