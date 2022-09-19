package com.example.taxi.ui.login.join

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.databinding.FragmentJoinBinding
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.isValidEmail
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class JoinFragment : BaseFragment<FragmentJoinBinding>(R.layout.fragment_join) {
    val authViewModel: AuthViewModel by viewModels()
    var isEachProvider = false

    override fun init() {
        setOnClickListeners()
    }

    private fun setOnClickListeners(){
        binding.buttonJoinLogin.setOnClickListener {
            if (validation()){
                isEachProvider = binding.switchJoinIsEachProvider.isChecked
                authViewModel.register(
                    email = binding.editTextJoinId.text.toString(),
                    password = binding.editTextJoinPw.text.toString(),
                    user = getUserObj()
                )
            }
        }
    }

    fun observer() {
        authViewModel.register.observe(viewLifecycleOwner) { state ->
            when(state){
                is UiState.Loading -> {
                    binding.buttonJoinLogin.setText("")
                    binding.progressBarJoinLoading.show()
                }
                is UiState.Failure -> {
                    binding.buttonJoinLogin.setText("Register")
                    binding.progressBarJoinLoading.hide()
                    state.error?.let { toast(it) }
                }
                is UiState.Success -> {
                    binding.buttonJoinLogin.setText("Register")
                    binding.progressBarJoinLoading.hide()
                    toast(state.data)
                    if(isEachProvider){
                        findNavController().navigate(R.id.action_joinFragment_to_providerHomeFragment)
                    }else{
                        findNavController().navigate(R.id.action_joinFragment_to_userHomeFragment)
                    }
                }
                else -> {}
            }
        }
    }

    private fun getUserObj(): User {
        return User(
            userSeq = "",
            name = binding.editTextJoinName.text.toString(),
            tel = binding.editTextJoinTel.text.toString()+"-"+binding.editTextJoinTel2.text.toString()+"-"+binding.editTextJoinTel3.text.toString(),
            userId = binding.editTextJoinId.text.toString(),
            isEachProvider = binding.switchJoinIsEachProvider.isChecked
        )
    }

    private fun validation(): Boolean {
        var isValid = true

        if (binding.editTextJoinName.text.isNullOrEmpty()){
            isValid = false
            toast("이름을 입력해 주세요.")
        }

        if (binding.editTextJoinTel.text.isNullOrEmpty()){
            isValid = false
            toast("핸드폰번호를 입력해 주세요.")
        }

        if (binding.editTextJoinTel2.text.isNullOrEmpty()){
            isValid = false
            toast("핸드폰번호를 입력해 주세요.")
        }

        if (binding.editTextJoinTel3.text.isNullOrEmpty()){
            isValid = false
            toast("핸드폰번호를 입력해 주세요.")
        }

        if (binding.editTextJoinId.text.isNullOrEmpty()){
            isValid = false
            toast("이메일을 입력해 주세요.")
        }else{
            if (!binding.editTextJoinId.text.toString().isValidEmail()){
                isValid = false
                toast("이메일을 양식에 맞게 입력해 주세요.")
            }
        }
        if (binding.editTextJoinPw.text.isNullOrEmpty()){
            isValid = false
            toast("비밀번호를 입력해 주세요.")
        }else{
            if (binding.editTextJoinPw.text.toString().length < 6){
                isValid = false
                toast("비밀번호를 6자리 이상 입력해 주세요.")
            }
        }
        if (binding.editTextJoinPwCheck.text.isNullOrEmpty()){
            isValid = false
            toast("비밀번호를 입력해 주세요.")
        }else{
            if (binding.editTextJoinPwCheck.text.toString().length < 6){
                isValid = false
                toast("비밀번호를 6자리 이상 입력해 주세요.")
            }
        }

        if(binding.editTextJoinPwCheck.text.toString() != binding.editTextJoinPw.text.toString()) {
            isValid = false
            toast("비밀번호 확인에 실패했습니다. 다시 입력해주세요.")
        }
        return isValid
    }
}