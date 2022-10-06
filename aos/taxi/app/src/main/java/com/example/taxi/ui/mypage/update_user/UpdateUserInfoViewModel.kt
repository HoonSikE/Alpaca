package com.example.taxi.ui.mypage.update_user

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.data.repository.UserInfoRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class UpdateUserInfoViewModel @Inject constructor(
    private val userinfoRepository : UserInfoRepository
): ViewModel() {
    private val _addressInfo = MutableLiveData<UiState<AddressInfo>>()
    val addressInfo: LiveData<UiState<AddressInfo>>
    get() = _addressInfo

    // 사진
    private val _addImageUpLoad = MutableLiveData<UiState<User>>()
    val addImageUpLoad: LiveData<UiState<User>>
        get() = _addImageUpLoad

    // Provider 확인
    private val _updateUserEachProvider = MutableLiveData<UiState<Boolean>>()
    val updateUserEachProvider: LiveData<UiState<Boolean>>
        get() = _updateUserEachProvider
    
    // 전화번호
    private val _updateUserTel = MutableLiveData<UiState<String>>()
    val updateUserTel: LiveData<UiState<String>>
        get() = _updateUserTel
    
    // 주소 삭제
    private val _deleteUserAddress = MutableLiveData<UiState<String>>()
    val deleteUserAddress: LiveData<UiState<String>>
        get() = _deleteUserAddress

    // 사진 삭제
    private val _deleteImage = MutableLiveData<UiState<String>>()
    val deleteImage: LiveData<UiState<String>>
        get() = _deleteImage

    fun addAddressInfo(addressInfo: AddressInfo){
        userinfoRepository.addAddressInfo(
            addressInfo = addressInfo
        ) {
            _addressInfo.value = it
        }
    }

    fun getAddressInfo(){
        userinfoRepository.getAddressInfo {
            _addressInfo.value = it
        }
    }

    fun addImageUpLoad(user: User){
        userinfoRepository.addImageUpLoad(
            user = user
        ) {
            _addImageUpLoad.value = it
        }
    }

    fun updateUserEachProvider(isEachProvider: Boolean){
        userinfoRepository.updateUserEachProvider(
            isEachProvider = isEachProvider
        ) {
            _updateUserEachProvider.value = it
        }
    }
    
    fun updateUserTel(tel: String){
        userinfoRepository.updateUserTel(
            tel = tel
        ) {
            _updateUserTel.value = it
        }
    }

    fun deleteUserAddress(){
        _deleteUserAddress.value = UiState.Loading
        userinfoRepository.deleteUserAddress { _deleteUserAddress.value = it }
    }

    fun deleteImage(){
        _deleteImage.value = UiState.Loading
        userinfoRepository.deleteImage { _deleteImage.value = it }
    }
}