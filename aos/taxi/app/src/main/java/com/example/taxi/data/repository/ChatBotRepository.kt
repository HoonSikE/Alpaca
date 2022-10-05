package com.example.taxi.data.repository

import com.example.taxi.utils.constant.UiState
import com.google.firebase.database.DataSnapshot

interface ChatBotRepository {
    fun checkChatBotMessage(message: String, result: (UiState<DataSnapshot>) -> Unit)
}