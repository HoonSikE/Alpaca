package com.example.taxi.data.repository

import androidx.fragment.app.FragmentActivity
import com.example.taxi.data.dto.user.User
import com.example.taxi.utils.constant.UiState
import com.facebook.AccessToken
import com.google.firebase.auth.FirebaseUser

interface AuthRepository {
    fun getCurrentUser(result: (FirebaseUser) -> Unit)
    fun registerUser(email: String, password: String, user: User, result: (UiState<String>) -> Unit)
    fun snsRegister(user: User, result: (UiState<String>) -> Unit)
//    fun facebookRegister(user: User, result: (UiState<String>) -> Unit)
//    fun githubRegister(user: User, result: (UiState<String>) -> Unit)
    fun phoneAuth(phoneNumber: String, activity: FragmentActivity, result: (String) -> Unit)
    fun ckeckPhoneAuth(verificationId: String, code: String, result: (UiState<String>) -> Unit)
    fun updateUserInfo(user: User, result: (UiState<String>) -> Unit)
    fun loginUser(email: String, password: String, result: (UiState<String>) -> Unit)
    fun googleLogin(idToken: String, result: (UiState<String>) -> Unit)
    fun facebookLogin(accessToken: AccessToken, result: (UiState<String>) -> Unit)
    fun githubLogin(activity: FragmentActivity, result: (UiState<String>) -> Unit)
    fun reauthPassword(existingPassword: String, result: (UiState<String>) -> Unit)
    fun updatePassword(newPassword: String, result: (UiState<String>) -> Unit)
    fun forgotPassword(email: String, result: (UiState<String>) -> Unit)
    fun logout(result: () -> Unit)
    fun storeSession(id: String, result: (User?) -> Unit)
    fun getSession(result: (User?) -> Unit)
    fun withDrawal(result: (UiState<String>) -> Unit)
    fun deleteUserInfo(result: (UiState<String>) -> Unit)
}