package com.example.taxi.data.repository

import android.util.Log
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore

class UserInfoRepositoryImpl(private val database: FirebaseFirestore) : UserInfoRepository{
    override fun getAddressInfo(result: (UiState<AddressInfo>) -> Unit) {
        database.collection(FireStoreCollection.USERADDRESSINFO).document(ApplicationClass.prefs.userSeq.toString())
            .get()
            .addOnSuccessListener { document ->
                if(document != null){
                    val userInfo = document.toObject(AddressInfo::class.java)
                    if(userInfo != null){
                        result.invoke(
                            UiState.Success(userInfo)
                        )
                    }
                }else {
                    Log.d("getUserInfo", "No such document")
                }

            }
            .addOnFailureListener {
                Log.d("getUserInfo", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }
    override fun addAddressInfo(addresInfo: AddressInfo, result: (UiState<AddressInfo>) -> Unit) {
        val document = database.collection(FireStoreCollection.USERADDRESSINFO).document(ApplicationClass.prefs.userSeq.toString())
        document
            .set(addresInfo)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(addresInfo)
                )
                Log.d("getAddressInfo", "Address has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("getAddress", "Address has been created fail")
            }
    }
}