package com.example.taxi.data.repository

import android.net.Uri
import android.util.Log
import androidx.core.net.toUri
import com.example.taxi.data.dto.common.InsideCarList
import com.example.taxi.data.dto.common.PhotoList
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore
import com.google.firebase.storage.FirebaseStorage

class UserInfoRepositoryImpl(private val database: FirebaseFirestore) : UserInfoRepository{
    override fun getAddressInfo(result: (UiState<AddressInfo>) -> Unit) {
        database.collection(FireStoreCollection.USERADDRESSINFO).document(ApplicationClass.prefs.userSeq.toString())
            .get()
            .addOnSuccessListener { document ->
                if(document != null){
                    val addressInfo = document.toObject(AddressInfo::class.java)
                    if(addressInfo != null){
                        result.invoke(
                            UiState.Success(addressInfo)
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
                Log.d("addAddressInfo", "Address has been added successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("addAddress", "Address has been added fail")
            }
    }

    override fun addImageUpLoad(user: User, result: (UiState<User>) -> Unit) {
        var storage = FirebaseStorage.getInstance()
        var imgFileName = user.userSeq + ".png"

        storage.reference.child("user_profiles").child(imgFileName)
            .putFile(user.profileImage.toUri())//어디에 업로드할지 지정
            .addOnSuccessListener { taskSnapshot ->
                taskSnapshot.metadata?.reference?.downloadUrl?.addOnSuccessListener { it ->
                    ApplicationClass.prefs.profileImage = it.toString()
                    // firestore 업데이트
                    database.collection(FireStoreCollection.USER)
                        .document(ApplicationClass.prefs.userSeq.toString())
                        .update("profileImage", it)
                        .addOnCompleteListener { task ->
                            Log.d("addImageUpLoad", "Image has been uploaded success")
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

    override fun updateUserTel(tel: String, result: (UiState<String>) -> Unit){
        val document = database.collection(FireStoreCollection.USER).document(ApplicationClass.prefs.userSeq.toString())
        document.update("tel", tel)
            .addOnSuccessListener {
                ApplicationClass.prefs.tel = tel
                result.invoke(
                    UiState.Success(tel)
                )
                Log.d("updateUserTel", "Tel has been updated successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateUserTel", "Tel has been updated fail")
            }
    }

    override fun deleteUserAddress(result: (UiState<String>) -> Unit){
        val document = database.collection(FireStoreCollection.USERADDRESSINFO).document(ApplicationClass.prefs.userSeq.toString())
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

    override fun deleteImage(result: (UiState<String>) -> Unit){
        var storage = FirebaseStorage.getInstance()
        var imgFileName = ApplicationClass.prefs.userSeq.toString() + ".png"

        storage.reference.child("user_profiles").child(imgFileName)
            .delete()
            .addOnSuccessListener {
                Log.d("addImageUpLoad", "Image has been uploaded successfully")
            }.addOnFailureListener{
                Log.d("addImageUpLoad", "Image has been uploaded fail")
            }
    }
    override fun getBoardedTaxiList(result: (UiState<BoardedTaxiList>) -> Unit) {
        database.collection(FireStoreCollection.BOARDEDTAXILIST).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                if(document != null){
//                    val map = document.toObject(Map::class.java)
                    val boardedTaxiList = document.toObject(BoardedTaxiList::class.java)
                    if(boardedTaxiList != null){
                        result.invoke(
                            UiState.Success(boardedTaxiList)
                        )
                    }
                }else {
                    Log.d("getBoardedTaxiList", "No such document")
                }

            }
            .addOnFailureListener {
                Log.d("getBoardedTaxiList", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun updateBoardedTaxiList(boardedTaxi: List<BoardedTaxi>, result: (UiState<List<BoardedTaxi>>) -> Unit){
        val document = database.collection(FireStoreCollection.BOARDEDTAXILIST).document(ApplicationClass.userId)
        document.update("taxiList", boardedTaxi)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(boardedTaxi)
                )
                Log.d("updateBoardedTaxiList", "BoardedTaxiList has been updated successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateBoardedTaxiList", "BoardedTaxiList has been updated fail")
            }
    }

    override fun getInsideCarList(result: (UiState<InsideCarList>) -> Unit) {
        database.collection(FireStoreCollection.INSIDECARLIST).document(ApplicationClass.userId)
            .get()
            .addOnSuccessListener { document ->
                if(document != null){
//                    val map = document.toObject(Map::class.java)
                    val insideCarList = document.toObject(InsideCarList::class.java)
                    if(insideCarList != null){
                        result.invoke(
                            UiState.Success(insideCarList)
                        )
                    }
                }else {
                    Log.d("getInsideCarList", "No such document")
                }

            }
            .addOnFailureListener {
                Log.d("getInsideCarList", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun addImageListUpLoad(count: Int, chk: Boolean, photoList: List<PhotoList>, result: (UiState<String>) -> Unit) {
        var storage = FirebaseStorage.getInstance()
        var list : MutableList<String> = mutableListOf()
        var cnt = 0
        for(i in photoList){
            if(i.carNumber == ApplicationClass.prefs.carNumber){
                if(chk){
                    for(j in 0 until i.start.size){
                        var imgFileName = ApplicationClass.userId + ApplicationClass.prefs.carNumber + j.toString() + ".png"
                        storage.reference.child("insideCarList").child(imgFileName)
                            .putFile(i.start[j].toUri())//어디에 업로드할지 지정
                            .addOnSuccessListener { taskSnapshot ->
                                taskSnapshot.metadata?.reference?.downloadUrl?.addOnSuccessListener { it ->
                                    list.add(it.toString())
                                    cnt++
                                    if(cnt == count){
                                        addList(list, photoList,chk)
                                    }
                                }
                            }.addOnFailureListener{
                                Log.d("addImageUpLoad", "Image has been uploaded fail")
                            }
                    }

                }else{
                    for(j in 0 until i.end.size){
                        var imgFileName = ApplicationClass.userId + ApplicationClass.prefs.carNumber + j.toString() + ".png"
                        storage.reference.child("insideCarList").child(imgFileName)
                            .putFile(i.end[j].toUri())//어디에 업로드할지 지정
                            .addOnSuccessListener { taskSnapshot ->
                                taskSnapshot.metadata?.reference?.downloadUrl?.addOnSuccessListener { it ->
                                    list.add(it.toString())
                                    cnt++
                                    if(cnt == count){
                                        addList(list, photoList,chk)
                                    }
                                }
                            }.addOnFailureListener{
                                Log.d("addImageUpLoad", "Image has been uploaded fail")
                            }
                    }
                }
                break
            }
        }
        UiState.Success("Image has been uploaded success")
    }

    fun addList(list : MutableList<String>, photoList: List<PhotoList>, chk: Boolean) {
        for(i in photoList) {
            if (i.carNumber == ApplicationClass.prefs.carNumber) {
                if(chk){
                    i.start = list
                }else{
                    i.end = list
                }
            }
        }
        database.collection(FireStoreCollection.INSIDECARLIST)
            .document(ApplicationClass.userId)
            .update("photoList", photoList)
            .addOnCompleteListener { task ->
                Log.d("addImageUpLoad", "Image has been uploaded success")
            }.addOnFailureListener { task ->
                Log.d("addImageUpLoad", "Image has been uploaded fail")
            }

    }
}