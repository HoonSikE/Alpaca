package com.example.taxi.ui.login

import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentEmailLoginBinding
import com.example.taxi.utils.constant.isValidEmail
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class EmailLoginFragment : BaseFragment<FragmentEmailLoginBinding>(R.layout.fragment_email_login) {

    val authViewModel: AuthViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
    }

    private fun setOnClickListeners(){
        binding.buttonEmailLoginLogin.setOnClickListener {
            if (validation()) {
                authViewModel.login(
                    email = binding.editTextEmailLoginId.text.toString(),
                    password = binding.editTextEmailLoginPw.text.toString()
                )
            }
        }

        binding.textEmailLoginFindID.setOnClickListener {
            findNavController().navigate(R.id.action_loginFragment_to_findIDFragment)
        }

        binding.textEmailLoginFindPW.setOnClickListener {
            findNavController().navigate(R.id.action_loginFragment_to_findPWFragment)
        }
    }

    private fun validation(): Boolean {
        var isValid = true

        if (binding.editTextEmailLoginId.text.isNullOrEmpty()){
            isValid = false
            toast("이메일을 입력해주세요.")
        }else{
            if (!binding.editTextEmailLoginId.text.toString().isValidEmail()){
                isValid = false
                toast("이메일 양식에 맞게 다시 입력해 주세요.")
            }
        }
        if (binding.editTextEmailLoginPw.text.isNullOrEmpty()){
            isValid = false
            toast("비밀번호를 입력해주세요.")
        }else{
            if (binding.editTextEmailLoginPw.text.toString().length < 6){
                isValid = false
                toast("비밀번호를 6자리 이상 입력해주세요.")
            }
        }
        return isValid
    }

}