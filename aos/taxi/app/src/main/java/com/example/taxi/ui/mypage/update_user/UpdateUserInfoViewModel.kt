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

    // 전화번호
    private val _updateUserTel = MutableLiveData<UiState<String>>()
    val updateUserTel: LiveData<UiState<String>>
        get() = _updateUserTel

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

    fun updateUserTel(tel: String){
        userinfoRepository.updateUserTel(
            tel = tel
        ) {
            _updateUserTel.value = it
        }
    }
}