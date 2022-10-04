package com.example.taxi.data.repository

import com.example.taxi.utils.constant.UiState
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ValueEventListener

class ChatBotRepositoryImpl(
) : ChatBotRepository {
    private val database = FirebaseDatabase.getInstance().reference

    override fun checkChatBotMessage(message: String, result: (UiState<DataSnapshot>) -> Unit) {
        database.child("ChatBot").child(message).addListenerForSingleValueEvent(object : ValueEventListener {
                override fun onCancelled(error: DatabaseError) {
                }
                override fun onDataChange(snapshot: DataSnapshot) {
                    result.invoke(
                        UiState.Success(snapshot)
                    )
                }
            })
    }
}