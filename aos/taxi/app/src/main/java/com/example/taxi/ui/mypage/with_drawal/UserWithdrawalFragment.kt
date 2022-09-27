package com.example.taxi.ui.mypage.with_drawal

import androidx.core.content.ContextCompat
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentUserWithDrawalBinding
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.ui.mypage.update_user.UpdateUserInfoViewModel
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UserWithdrawalFragment : BaseFragment<FragmentUserWithDrawalBinding>(R.layout.fragment_user_with_drawal) {
    val authViewModel: AuthViewModel by viewModels()
    val updateUserInfoViewModel: UpdateUserInfoViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
    }
    private fun setOnClickListeners() {
        binding.buttonUserWithdrawal.setOnClickListener{
            if(binding.switchUserWithdrawalSwitch.isChecked){
                val dialog = DialogUserWithDrawalFragment()
                dialog.setOnOKClickedListener { content ->
                    authViewModel.deleteUserInfo{}
//                    updateUserInfoViewModel.deleteImage{}
//                    updateUserInfoViewModel.deleteUserAddress{}
                    authViewModel.withDrawal {
                        toast("회원탈퇴가 완료되었습니다.")
                        findNavController().navigate(R.id.action_userWithdrawalFragment_to_loginFragment)
                    }
                }
                dialog.show(childFragmentManager, "user withdrawal")
            }else{
                toast("회원탈퇴 안내를 확인해주십시오.")
            }
        }
    }
}