package com.example.taxi.data.repository

import android.util.Log
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.mypage.FavoritesDto
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.DestinationDto
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore


class DestinationRepositoryImpl(
    private val database: FirebaseFirestore
) : DestinationRepository {

    override fun getDestination(result: (UiState<List<Destination>>) -> Unit) {
        database.collection(FireStoreCollection.DESTINATION).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                Log.d("getDestination", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val destinationDto = document.toObject(DestinationDto::class.java)
                    if(destinationDto != null){
                        val destinations = destinationDto.destination
                        val newDestination = mutableListOf<Destination>()
                        if(destinations != null){
                            for(des in destinations){
                                val destination = Destination(des.address, des.latitude, des.addressName, des.longitude, des.count)
                                newDestination.add(destination)
                            }
                            result.invoke(
                                UiState.Success(newDestination)
                            )
                        }
                    }
                } else {
                    Log.d("getDestination", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getDestination", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun addDestination(destination: Destination, result: (UiState<Destination>) -> Unit) {
        val document = database.collection(FireStoreCollection.DESTINATION).document(ApplicationClass.userId)
        document
            .set(destination)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(destination)
                )
                Log.d("getDestination", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("getDestination", "Destination has been created fail")
            }
    }

    override fun getFavorites(result: (UiState<List<Favorites>>) -> Unit) {
        database.collection(FireStoreCollection.FAVORITES).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                Log.d("getFavorites", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val favoritesDto = document.toObject(FavoritesDto::class.java)
                    if(favoritesDto != null){
                        val favorites = favoritesDto.favorites
                        val newFavorite = mutableListOf<Favorites>()
                        if(favorites != null){
                            for(fav in favorites){
                                val favorite = Favorites(fav.address, fav.latitude, fav.addressName, fav.longitude)
                                newFavorite.add(favorite)
                            }
                            result.invoke(
                                UiState.Success(newFavorite)
                            )
                        }
                    }
                } else {
                    Log.d("getFavorites", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getFavorites", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun addFavorites(destination: Destination, result: (UiState<Favorites>) -> Unit) {
        TODO("Not yet implemented")
    }

}