package com.example.taxi.data.repository

import com.example.taxi.data.dto.mypage.ChatModel
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.utils.constant.UiState
import com.google.firebase.database.DataSnapshot

interface PersonalChatRepository {
    fun checkChatRoom(startUserName: String, destinationUserName: String, result: (UiState<DataSnapshot>) -> Unit)
    fun createChatRooms(chatModel : ChatModel, result: (UiState<DataSnapshot>) -> Unit)
    fun addComment(chatRoomUid : String, comment: ChatModel.CommentModel, result: (UiState<String>) -> Unit)
    fun getComment(destinationUserName: String, chatRoomUid : String, result: (UiState<DataSnapshot>) -> Unit)
}