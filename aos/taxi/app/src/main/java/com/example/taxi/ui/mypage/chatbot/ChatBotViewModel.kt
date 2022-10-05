package com.example.taxi.ui.mypage.chatbot

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.repository.ChatBotRepository
import com.example.taxi.utils.constant.UiState
import com.google.firebase.database.DataSnapshot
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class ChatBotViewModel @Inject constructor(
    private val repository: ChatBotRepository
): ViewModel() {
    private val _checkChatBotMessage = MutableLiveData<UiState<DataSnapshot>>()
    val checkChatBotMessage: LiveData<UiState<DataSnapshot>>
        get() = _checkChatBotMessage

    fun checkChatBotMessage(message: String) {
        _checkChatBotMessage.value = UiState.Loading
        repository.checkChatBotMessage(message) { _checkChatBotMessage.value = it }
    }

}