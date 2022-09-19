package com.example.taxi.ui.home.user

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.repository.DestinationRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class UserHomeViewModel @Inject constructor(
    private val repository: DestinationRepository
): ViewModel() {
    
    private val _destinations = MutableLiveData<UiState<List<Destination>>>()
    val destinations: LiveData<UiState<List<Destination>>>
        get() = _destinations

    private val _addNote = MutableLiveData<UiState<Pair<Destination,String>>>()
    val addNote: LiveData<UiState<Pair<Destination,String>>>
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