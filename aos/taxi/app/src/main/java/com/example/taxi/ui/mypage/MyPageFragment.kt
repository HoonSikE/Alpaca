package com.example.taxi.ui.mypage

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.databinding.FragmentMyPageBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.google.firebase.firestore.FirebaseFirestore
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {
    val authViewModel: AuthViewModel by viewModels()

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initData() {
        val profileImage = ApplicationClass.prefs?.profileImage
        if (profileImage != "")
            Glide.with(this).load(profileImage).into(binding.imageMyPageProfile)

        binding.textMyPageName.text = ApplicationClass.prefs.name + "님, 안녕하세요!"
        binding.textMyPageCount.text = "회"
        binding.textMyPageClass.text = "None"
    }

    private fun setOnClickListeners() {
        binding.textUpdateUserInfo.setOnClickListener {
            findNavController().navigate(R.id.action_myPageFragment_to_updateUserInfoFragment)
        }
        binding.textUpdatePassword.setOnClickListener {
            findNavController().navigate(R.id.action_myPageFragment_to_updatePasswordFragment)
        }
        binding.textUpdateUserLogout.setOnClickListener {
            authViewModel.logout {
                findNavController().navigate(R.id.action_myPageFragment_to_loginFragment)
            }
        }
        binding.textUserWithdrawal.setOnClickListener {
            findNavController().navigate(R.id.action_myPageFragment_to_userWithdrawalFragment)
        }
    }

    private fun observerData() {

    }
}