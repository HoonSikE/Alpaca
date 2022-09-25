package com.example.taxi.ui.mypage.update_password

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentUpdatePasswordBinding
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.isValidEmail
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UpdatePasswordFragment : BaseFragment<FragmentUpdatePasswordBinding>(R.layout.fragment_update_password) {
    val authViewModel: AuthViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
        observerData()
    }
    private fun setOnClickListeners(){
        binding.buttonUserWithdrawal.setOnClickListener{
            if (validation()){
                authViewModel.reauthPassword(
                    existingPassword = binding.editTextExistingPassword.text.toString()
                )
            }
        }
    }
    private fun observerData() {
        authViewModel.reauthPassword.observe(viewLifecycleOwner){ state ->
            when(state){
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    toast("기존 비밀번호가 일치하지 않습니다!")
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    toast("기존 비밀번호가 일치합니다!")
                    authViewModel.updatePassword(
                        newPassword = binding.editTextUpdatePassword2.text.toString()
                    )
                }
            }
        }
        authViewModel.updatePassword.observe(viewLifecycleOwner){ state ->
            when(state){
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    toast("비밀번호 변경이 완료되었습니다.ㅋㅌ!")
                    findNavController().navigate(R.id.action_updatePasswordFragment_to_myPageFragment)
                }
            }

        }
    }
    private fun validation(): Boolean {
        if (binding.editTextExistingPassword.text.isNullOrEmpty()){
            toast("기존 비밀번호를 입력해 주세요.")
            return false
        }else{
            if (binding.editTextExistingPassword.text.toString().length < 6){
                toast("비밀번호를 6자리 이상 입력해 주세요.")
                return false
            }
        }

        if (binding.editTextUpdatePassword1.text.isNullOrEmpty()){
            toast("변결할 비밀번호를 입력해 주세요.")
            return false
        }else{
            if (binding.editTextUpdatePassword1.text.toString().length < 6){
                toast("비밀번호를 6자리 이상 입력해 주세요.")
                return false
            }
        }
        if (binding.editTextUpdatePassword2.text.isNullOrEmpty()){
            toast("변경할 비밀번호를 재입력해 주세요.")
            return false
        }else{
            if (binding.editTextUpdatePassword2.text.toString().length < 6){
                toast("비밀번호를 6자리 이상 입력해 주세요.")
                return false
            }
        }

        if(binding.editTextUpdatePassword1.text.toString() != binding.editTextUpdatePassword2.text.toString()) {
            toast("비밀번호가 일치하지 않습니다. 다시 입력해주세요.")
            return false
        }
        return true
    }
}