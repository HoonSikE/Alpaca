package com.example.taxi.data.repository

import android.util.Log
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore

class ProviderRepositoryImpl(
    private val database: FirebaseFirestore
) : ProviderRepository {
    override fun getProvider(result: (UiState<Provider>) -> Unit) {
        database.collection(FireStoreCollection.PROVIDER).document(ApplicationClass.prefs.providerId.toString())
            .get()
            .addOnSuccessListener { document ->
                Log.d("getProvider", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val provider = document.toObject(Provider::class.java)
                    if(provider != null){
                        result.invoke(
                            UiState.Success(provider)
                        )
                    }
                } else {
                    Log.d("getProvider", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getProvider", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun updateProvider(provider: ProviderCar, result: (UiState<ProviderCar>) -> Unit) {
        val document = database.collection(FireStoreCollection.PROVIDER).document(ApplicationClass.prefs.providerId.toString())
        document
            .update("car",provider)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(provider)
                )
                Log.d("updateProvider", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateProvider", "Destination has been created fail")
            }
    }

    override fun updateRevenue(revenue: Int, result: (UiState<Int>) -> Unit) {
        val document = database.collection(FireStoreCollection.PROVIDER).document(ApplicationClass.prefs.providerId.toString())
        document
            .update("revenue",revenue)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(revenue)
                )
                Log.d("updateRevenue", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateRevenue", "Destination has been created fail")
            }
    }

    override fun getUserList(result: (UiState<UserList>) -> Unit) {
        database.collection(FireStoreCollection.USERLIST).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                Log.d("getUserList", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val userList = document.toObject(UserList::class.java)
                    if(userList != null){
                        result.invoke(
                            UiState.Success(userList)
                        )
                    }
                } else {
                    Log.d("getUserList", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getUserList", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }
}