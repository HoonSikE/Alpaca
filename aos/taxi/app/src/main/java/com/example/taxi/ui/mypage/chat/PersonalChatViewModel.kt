package com.example.taxi.ui.mypage.chat

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.mypage.ChatModel
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.repository.PersonalChatRepository
import com.example.taxi.data.repository.ProviderRepository
import com.example.taxi.data.repository.RouteRepository
import com.example.taxi.utils.constant.UiState
import com.google.firebase.database.DataSnapshot
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class PersonalChatViewModel @Inject constructor(
    private val repository: PersonalChatRepository
): ViewModel() {
    private val _checkChatRoom = MutableLiveData<UiState<DataSnapshot>>()
    val checkChatRoom: LiveData<UiState<DataSnapshot>>
        get() = _checkChatRoom

    private val _createChatRooms = MutableLiveData<UiState<DataSnapshot>>()
    val createChatRooms: LiveData<UiState<DataSnapshot>>
        get() = _createChatRooms

    private val _addComment = MutableLiveData<UiState<String>>()
    val addComment: LiveData<UiState<String>>
        get() = _addComment

    private val _getComment = MutableLiveData<UiState<DataSnapshot>>()
    val getComment: LiveData<UiState<DataSnapshot>>
        get() = _getComment

    fun checkChatRoom(startUserName: String, destinationUserName: String) {
        _checkChatRoom.value = UiState.Loading
        repository.checkChatRoom(startUserName, destinationUserName) { _checkChatRoom.value = it }
    }

    fun createChatRooms(chatModel: ChatModel) {
        _createChatRooms.value = UiState.Loading
        repository.createChatRooms(chatModel) { _createChatRooms.value = it }
    }

    fun addComment(chatRoomUid : String, comment: ChatModel.CommentModel) {
        _addComment.value = UiState.Loading
        repository.addComment(chatRoomUid, comment) { _addComment.value = it }
    }

    fun getComment(destinationUserName: String, chatRoomUid : String) {
        _getComment.value = UiState.Loading
        repository.getComment(destinationUserName, chatRoomUid) { _getComment.value = it }
    }
}