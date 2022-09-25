package com.example.taxi.ui.mypage.update_password

import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentUpdatePasswordBinding
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.isValidEmail
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UpdatePasswordFragment : BaseFragment<FragmentUpdatePasswordBinding>(R.layout.fragment_update_password) {
    val authViewModel: AuthViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
    }
    private fun setOnClickListeners(){
        binding.buttonUserWithdrawal.setOnClickListener{
            if (validation()){
                authViewModel.updatePassword(
                    newPassword = binding.editTextUpdatePassword2.text.toString()
                )
                findNavController().navigate(R.id.action_updatePasswordFragment_to_myPageFragment)
            }
        }
    }
    private fun validation(): Boolean {
        var isValid = true

        if (binding.editTextUpdatePassword1.text.isNullOrEmpty()){
            isValid = false
            toast("변결할 비밀번호를 입력해 주세요.")
        }else{
            if (binding.editTextUpdatePassword1.text.toString().length < 6){
                isValid = false
                toast("비밀번호를 6자리 이상 입력해 주세요.")
            }
        }
        if (binding.editTextUpdatePassword2.text.isNullOrEmpty()){
            isValid = false
            toast("변경할 비밀번호를 재입력해 주세요.")
        }else{
            if (binding.editTextUpdatePassword2.text.toString().length < 6){
                isValid = false
                toast("비밀번호를 6자리 이상 입력해 주세요.")
            }
        }

        if(binding.editTextUpdatePassword1.text.toString() != binding.editTextUpdatePassword2.text.toString()) {
            isValid = false
            toast("비밀번호가 일치하지 않습니다. 다시 입력해주세요.")
        }
        return isValid
    }
}