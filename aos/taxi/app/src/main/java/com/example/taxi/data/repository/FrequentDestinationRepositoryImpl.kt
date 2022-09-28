package com.example.taxi.data.repository

import android.util.Log
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
import com.google.firebase.firestore.FirebaseFirestore


class FrequentDestinationRepositoryImpl(
    private val database: FirebaseFirestore
) : FrequentDestinationRepository {

    override fun getDestination(result: (UiState<List<FrequentDestination>>) -> Unit) {
        database.collection(FireStoreCollection.DESTINATION).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                Log.d("getDestination", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val frequentDestinationDto = document.toObject(FrequentDestinationDto::class.java)
                    if(frequentDestinationDto != null){
                        val destinations = frequentDestinationDto.frequentDestination
                        val newFrequentDestination = mutableListOf<FrequentDestination>()
                        if(destinations != null){
                            for(des in destinations){
                                val frequentDestination = FrequentDestination(des.address, des.latitude, des.count, des.addressName, des.longitude)
                                newFrequentDestination.add(frequentDestination)
                            }
                            result.invoke(
                                UiState.Success(newFrequentDestination)
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

    override fun updateDestination(provider: List<FrequentDestination>, result: (UiState<List<FrequentDestination>>) -> Unit) {
        val document = database.collection(FireStoreCollection.DESTINATION).document(ApplicationClass.userId)
        document
            .update("destination",provider)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(provider)
                )
                Log.d("updateDestination", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateDestination", "Destination has been created fail")
            }
    }

    override fun addDestination(frequentDestination: FrequentDestination, result: (UiState<FrequentDestination>) -> Unit) {
        val document = database.collection(FireStoreCollection.DESTINATION).document(ApplicationClass.userId)
        document
            .set(frequentDestination)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(frequentDestination)
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

    override fun addFavorites(
        frequentDestination: FrequentDestination,
        result: (UiState<Favorites>) -> Unit
    ) {
        TODO("Not yet implemented")
    }

    override fun getLastDestination(result: (UiState<List<Destination>>) -> Unit) {
        database.collection(FireStoreCollection.LASTDESTINATION).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                Log.d("getLastDestination", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val destinationDto = document.toObject(DestinationDto::class.java)
                    if(destinationDto != null){
                        val destinations = destinationDto.destination
                        val newDestination = mutableListOf<Destination>()
                        if(destinations != null){
                            for(des in destinations){
                                val destination = Destination(des.address, des.latitude, des.addressName, des.longitude)
                                newDestination.add(destination)
                            }
                            result.invoke(
                                UiState.Success(newDestination)
                            )
                        }
                    }
                } else {
                    Log.d("getLastDestination", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getLastDestination", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun updateLastDestination(provider: List<Destination>, result: (UiState<List<Destination>>) -> Unit) {
        val document = database.collection(FireStoreCollection.LASTDESTINATION).document(ApplicationClass.userId)
        document
            .update("destination",provider)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(provider)
                )
                Log.d("updateDestination", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateDestination", "Destination has been created fail")
            }
    }

}