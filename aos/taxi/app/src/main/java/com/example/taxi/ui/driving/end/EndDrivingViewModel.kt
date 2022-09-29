package com.example.taxi.ui.driving.end

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.common.InsideCarList
import com.example.taxi.data.dto.common.PhotoList
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.data.repository.UserInfoRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class EndDrivingViewModel @Inject constructor(
    private val userinfoRepository : UserInfoRepository
): ViewModel() {
    private val _boardedTaxiList = MutableLiveData<UiState<BoardedTaxiList>>()
    val boardedTaxiList: LiveData<UiState<BoardedTaxiList>>
        get() = _boardedTaxiList

    private val _updateBoardedTaxiList = MutableLiveData<UiState<List<BoardedTaxi>>>()
    val updateBoardedTaxiList: LiveData<UiState<List<BoardedTaxi>>>
        get() = _updateBoardedTaxiList

    private val _insideCarList = MutableLiveData<UiState<InsideCarList>>()
    val insideCarList: LiveData<UiState<InsideCarList>>
        get() = _insideCarList

    private val _photoList = MutableLiveData<UiState<String>>()
    val photoList: LiveData<UiState<String>>
        get() = _photoList

    fun getBoardedTaxiList() {
        _boardedTaxiList.value = UiState.Loading
        userinfoRepository.getBoardedTaxiList { _boardedTaxiList.value = it }
    }

    fun updateBoardedTaxiList(taxiList: List<BoardedTaxi>){
        _updateBoardedTaxiList.value = UiState.Loading
        userinfoRepository.updateBoardedTaxiList(taxiList) { _updateBoardedTaxiList.value = it }
    }

    fun getInsideCarList() {
        _insideCarList.value = UiState.Loading
        userinfoRepository.getInsideCarList { _insideCarList.value = it }
    }
    fun addImageListUpLoad(count: Int,chk: Boolean, photoList: List<PhotoList>) {
        _photoList.value = UiState.Loading
        userinfoRepository.addImageListUpLoad(count,chk,photoList) { _photoList.value = it }
    }
}