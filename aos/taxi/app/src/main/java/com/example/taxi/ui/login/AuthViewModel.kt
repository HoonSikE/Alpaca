package com.example.taxi.ui.login

import android.net.Uri
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentActivity
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.data.repository.AuthRepository
import com.example.taxi.data.repository.UserInfoRepository
import com.example.taxi.utils.constant.UiState
import com.facebook.AccessToken
import com.google.firebase.auth.FirebaseUser
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

    private val _snsRegister = MutableLiveData<UiState<String>>()
    val snsRegister: LiveData<UiState<String>>
        get() = _snsRegister

    private val _phoneAuth = MutableLiveData<String>()
    val phoneAuth: LiveData<String>
        get() = _phoneAuth

    private val _ckeckPhoneAuth = MutableLiveData<UiState<String>>()
    val ckeckPhoneAuth: LiveData<UiState<String>>
        get() = _ckeckPhoneAuth

    private val _login = MutableLiveData<UiState<String>>()
    val login: LiveData<UiState<String>>
        get() = _login

    private val _googleLogin = MutableLiveData<UiState<String>>()
    val googleLogin: LiveData<UiState<String>>
        get() = _googleLogin

    private val _facebookLogin = MutableLiveData<UiState<String>>()
    val facebookLogin: LiveData<UiState<String>>
        get() = _facebookLogin

    private val _githubLogin = MutableLiveData<UiState<String>>()
    val githubLogin: LiveData<UiState<String>>
        get() = _githubLogin

    private val _reauthPassword = MutableLiveData<UiState<String>>()
    val reauthPassword: LiveData<UiState<String>>
        get() = _reauthPassword

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

    // 회원 탈퇴
    private val _withDrawal = MutableLiveData<UiState<String>>()
    val withDrawal: LiveData<UiState<String>>
        get() = _withDrawal

    fun getCurrentUser(result: (FirebaseUser) -> Unit){
        repository.getCurrentUser(result)
    }

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

    fun snsRegister(user : User) {
        _snsRegister.value = UiState.Loading
        repository.snsRegister(
            user = user
        ) { _snsRegister.value = it }
    }

    fun login(
        email: String,
        password: String
    ) {
        _login.value = UiState.Loading
        repository.loginUser(
            email = email,
            password = password
        ){
            _login.value = it
        }
    }

    fun facebookLogin(accessToken: AccessToken) {
        _facebookLogin.value = UiState.Loading
        repository.facebookLogin(accessToken){
            _facebookLogin.value = it
        }
    }

    fun googleLogin(idToken: String) {
        _googleLogin.value = UiState.Loading
        repository.googleLogin(idToken){
            _googleLogin.value = it
        }
    }

    fun githubLogin(activity: FragmentActivity) {
        _githubLogin.value = UiState.Loading
        repository.githubLogin(activity){
            _githubLogin.value = it
        }
    }

    fun reauthPassword(existingPassword: String) {
        _reauthPassword.value = UiState.Loading
        repository.reauthPassword(existingPassword){
            _reauthPassword.value = it
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

    fun phoneAuth(phoneNumber: String, activity: FragmentActivity){
        repository.phoneAuth(
            phoneNumber = phoneNumber,
            activity = activity
        ){
            _phoneAuth.value = it
        }
    }

    fun ckeckPhoneAuth(verificationId: String, code: String){
        repository.ckeckPhoneAuth(
            verificationId = verificationId,
            code = code
        ){
            _ckeckPhoneAuth.value = it
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

    fun withDrawal(){
        _withDrawal.value = UiState.Loading
        repository.withDrawal { _withDrawal.value = it }
    }

    fun deleteUserInfo() {
        _deleteUserInfo.value = UiState.Loading
        repository.deleteUserInfo { _deleteUserInfo.value = it }
    }
}