package com.example.taxi.data.repository

import android.util.Log
import androidx.core.net.toUri
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore
import com.google.firebase.storage.FirebaseStorage

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

    override fun addProvider(provider: Provider, result: (UiState<Provider>) -> Unit){
        if(provider.car?.carImage != ""){
            var storage = FirebaseStorage.getInstance()
            var imgFileName = ApplicationClass.prefs.providerId + ".png"

            provider.car?.carImage?.let {
                storage.getReference().child("provider_car_profiles").child(imgFileName)
                    .putFile(it.toUri())//어디에 업로드할지 지정
                    .addOnSuccessListener { taskSnapshot ->
                        taskSnapshot.metadata?.reference?.downloadUrl?.addOnSuccessListener { it ->
                            ApplicationClass.prefs.carImage = it.toString()
                            provider.car?.carImage = it.toString()
                            // firestore 업데이트
                            database.collection(FireStoreCollection.PROVIDER)
                                .document(ApplicationClass.prefs.providerId.toString())
                                .set(provider)
                                .addOnSuccessListener { task ->
                                    result.invoke(UiState.Success(provider))
                                }.addOnFailureListener { task ->
                                    result.invoke(
                                        UiState.Failure(
                                            task.localizedMessage
                                        )
                                    )
                                }
                        }
                    }.addOnFailureListener{
                        Log.d("addImageUpLoad", "Image has been uploaded fail")
                    }
            }
        }else{
            val document = database.collection(FireStoreCollection.PROVIDER).document(ApplicationClass.prefs.providerId.toString())
            document
                .set(provider)
//            .update("car",provider)
                .addOnSuccessListener {
                    result.invoke(UiState.Success(provider))
                    Log.d("updateProvider", "Destination has been created successfully")
                }
                .addOnFailureListener {
                    result.invoke(
                        UiState.Failure(it.localizedMessage)
                    )
                    Log.d("updateProvider", "Destination has been created fail")
                }
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

    override fun deleteProvider(result: (UiState<String>) -> Unit){
        val document = database.collection(FireStoreCollection.PROVIDER).document(ApplicationClass.userId)
        document.delete()
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success("User has been deleted successfully")
                )
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
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
        database.collection(FireStoreCollection.USERLIST).document(ApplicationClass.prefs.providerId.toString())
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

    override fun updateUserList(userList: UserList, result: (UiState<UserList>) -> Unit) {
        val document = database.collection(FireStoreCollection.USERLIST).document(ApplicationClass.prefs.providerId.toString())
        document
            .update("user",userList)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(userList)
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
}