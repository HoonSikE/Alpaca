package com.example.taxi.ui.find.pw

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentFindPwBinding
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class FindPWFragment : BaseFragment<FragmentFindPwBinding>(R.layout.fragment_find_pw) {
    val authViewModel: AuthViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
        observerData()
    }

    private fun setOnClickListeners() {
        binding.buttonFindPw.setOnClickListener {
            authViewModel.forgotPassword(
                email = binding.editTextFindPw.text.toString()
            )
        }
    }

    private fun observerData() {
        binding.imgFindPwBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        authViewModel.forgotPassword.observe(viewLifecycleOwner){ state ->
            when(state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    toast("이메일 전송에 실패했습니다.!")
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    toast("이메일 전송에 성공했습니다.!")
                    findNavController().navigate(R.id.action_findPWFragment_to_loginFragment)
                }
            }
        }
    }
}