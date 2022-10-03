package com.example.taxi.data.repository

import android.os.Handler
import android.util.Log
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.taxi.data.dto.mypage.Chat
import com.example.taxi.data.dto.mypage.ChatModel
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.mypage.FavoritesDto
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.DestinationDto
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.data.dto.user.destination.FrequentDestinationDto
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ValueEventListener
import com.google.firebase.database.ktx.getValue
import com.google.firebase.firestore.FirebaseFirestore
import org.w3c.dom.Comment


class PersonalChatRepositoryImpl(
) : PersonalChatRepository {
    private val database = FirebaseDatabase.getInstance().reference

    override fun checkChatRoom(startUserName: String, destinationUserName: String, result: (UiState<DataSnapshot>) -> Unit) {
        database.child("PersonalChatRooms").orderByChild("users/$startUserName").equalTo(true)
            .addListenerForSingleValueEvent(object : ValueEventListener {
                override fun onCancelled(error: DatabaseError) {
                }
                override fun onDataChange(snapshot: DataSnapshot) {
                    result.invoke(
                        UiState.Success(snapshot)
                    )
                }
            })
    }

    override fun createChatRooms(chatModel: ChatModel, result: (UiState<DataSnapshot>) -> Unit){
        // 채팅방 생성
        database.child("PersonalChatRooms").push().setValue(chatModel)
            .addOnSuccessListener {
                database.child("PersonalChatRooms").orderByChild("users/${chatModel.users}").equalTo(true)
                    .addListenerForSingleValueEvent(object : ValueEventListener {
                        override fun onCancelled(error: DatabaseError) {
                        }
                        override fun onDataChange(snapshot: DataSnapshot) {
                            result.invoke(
                                UiState.Success(snapshot)
                            )
                        }
                    })
            }.addOnFailureListener{
                result.invoke(UiState.Failure("createChatRooms Failed"))
            }
    }

    override fun addComment(chatRoomUid : String, comment: ChatModel.CommentModel, result: (UiState<String>) -> Unit){
        database.child("PersonalChatRooms").child(chatRoomUid.toString()).child("comments").push().setValue(comment)
            .addOnSuccessListener {
                result.invoke(UiState.Failure("addComment Successed"))
            }.addOnFailureListener{
                result.invoke(UiState.Failure("addComment Failed"))
            }
    }

    override fun getComment(destinationUserName: String, chatRoomUid : String, result: (UiState<DataSnapshot>) -> Unit){
        database.child("users").child(destinationUserName).addListenerForSingleValueEvent(object : ValueEventListener{
            override fun onCancelled(error: DatabaseError) {
            }
            override fun onDataChange(snapshot: DataSnapshot) {
                database.child("PersonalChatRooms").child(chatRoomUid).child("comments").addValueEventListener(object : ValueEventListener{
                    override fun onCancelled(error: DatabaseError) {
                    }
                    override fun onDataChange(snapshot: DataSnapshot) {
                        result.invoke(
                            UiState.Success(snapshot)
                        )
                    }
                })
            }
        })
    }
}