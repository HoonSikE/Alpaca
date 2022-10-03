package com.example.taxi.data.repository

import androidx.fragment.app.FragmentActivity
import com.example.taxi.data.dto.user.User
import com.example.taxi.utils.constant.UiState

interface AuthRepository {
    fun registerUser(email: String, password: String, user: User, result: (UiState<String>) -> Unit)
    fun phoneAuth(phoneNumber: String, activity: FragmentActivity, result: (String) -> Unit)
    fun ckeckPhoneAuth(verificationId: String, code: String, result: (UiState<String>) -> Unit)
    fun updateUserInfo(user: User, result: (UiState<String>) -> Unit)
    fun loginUser(email: String, password: String, result: (UiState<String>) -> Unit)
    fun reauthPassword(existingPassword: String, result: (UiState<String>) -> Unit)
    fun updatePassword(newPassword: String, result: (UiState<String>) -> Unit)
    fun forgotPassword(email: String, result: (UiState<String>) -> Unit)
    fun logout(result: () -> Unit)
    fun storeSession(id: String, result: (User?) -> Unit)
    fun getSession(result: (User?) -> Unit)
    fun withDrawal(result: () -> Unit)
    fun deleteUserInfo(result: (UiState<String>) -> Unit)
}