package com.example.taxi.ui.mypage.update_user

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
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

    fun getAddressInfo(){
        userinfoRepository.getAddressInfo {
            _addressInfo.value = it
        }
    }
}