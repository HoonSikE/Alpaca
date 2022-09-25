package com.example.taxi.ui.login

import android.net.Uri
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.data.repository.AuthRepository
import com.example.taxi.data.repository.UserInfoRepository
import com.example.taxi.utils.constant.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class AuthViewModel @Inject constructor(
    private val repository: AuthRepository,
    private val userinfoRepository : UserInfoRepository
): ViewModel() {

    private val _register = MutableLiveData<UiState<String>>()
    val register: LiveData<UiState<String>>
            get() = _register

    private val _login = MutableLiveData<UiState<String>>()
    val login: LiveData<UiState<String>>
        get() = _login

    private val _updatePassword = MutableLiveData<UiState<String>>()
    val updatePassword: LiveData<UiState<String>>
        get() = _updatePassword

    private val _forgotPassword = MutableLiveData<UiState<String>>()
    val forgotPassword: LiveData<UiState<String>>
        get() = _forgotPassword

    // 주소
    private val _addressInfo = MutableLiveData<UiState<AddressInfo>>()
    val addressInfo: LiveData<UiState<AddressInfo>>
        get() = _addressInfo

    // 사진
    private val _addImageUpLoad = MutableLiveData<UiState<User>>()
    val addImageUpLoad: LiveData<UiState<User>>
        get() = _addImageUpLoad

    // 유저 삭제
    private val _deleteUserInfo = MutableLiveData<UiState<String>>()
    val deleteUserInfo: LiveData<UiState<String>>
        get() = _deleteUserInfo

    fun register(
        email: String,
        password: String,
        user: User
    ) {
        _register.value = UiState.Loading
        repository.registerUser(
            email = email,
            password = password,
            user = user
        ) { _register.value = it }
    }

    fun login(
        email: String,
        password: String
    ) {
        _login.value = UiState.Loading
        repository.loginUser(
            email,
            password
        ){
            _login.value = it
        }
    }

    fun updatePassword(newPassword: String) {
        _updatePassword.value = UiState.Loading
        repository.updatePassword(newPassword){
            _updatePassword.value = it
        }
    }

    fun forgotPassword(email: String) {
        _forgotPassword.value = UiState.Loading
        repository.forgotPassword(email){
            _forgotPassword.value = it
        }
    }

    fun logout(result: () -> Unit){
        repository.logout(result)
    }

    fun getSession(result: (User?) -> Unit){
        repository.getSession(result)
    }

    fun addAddressInfo(addressInfo: AddressInfo){
        userinfoRepository.addAddressInfo(
            addressInfo = addressInfo
        ) {
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

    fun withDrawal(result: () -> Unit){
        repository.withDrawal(result)
    }

    fun deleteUserInfo(result: () -> Unit){
        repository.deleteUserInfo(
        ) {
            _deleteUserInfo.value = it
        }
    }
}