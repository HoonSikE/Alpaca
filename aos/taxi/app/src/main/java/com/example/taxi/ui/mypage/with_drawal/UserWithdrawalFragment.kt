package com.example.taxi.ui.mypage.with_drawal

import android.util.Log
import androidx.core.content.ContextCompat
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentUserWithDrawalBinding
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.ui.mypage.update_user.UpdateUserInfoViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UserWithdrawalFragment : BaseFragment<FragmentUserWithDrawalBinding>(R.layout.fragment_user_with_drawal) {
    val authViewModel: AuthViewModel by viewModels()
    val updateUserInfoViewModel: UpdateUserInfoViewModel by viewModels()
    val userHomeViewModel : UserHomeViewModel by viewModels()
    val providerViewModel : ProviderViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
        observerData()
    }

    private fun setOnClickListeners() {
        binding.imgUserWithdrawalBack.setOnClickListener {
            requireActivity().onBackPressed()
        }
        binding.buttonUserWithdrawal.setOnClickListener {
            if (binding.switchUserWithdrawalSwitch.isChecked) {
                val dialog = UserWithDrawalDialogFragment()
                dialog.setOnOKClickedListener { content ->
                    authViewModel.deleteUserInfo()
                    updateUserInfoViewModel.deleteImage()
                    updateUserInfoViewModel.deleteUserAddress()
                    userHomeViewModel.deleteFavorites()
                    providerViewModel.deleteProvider()
                    authViewModel.withDrawal()
                }
                dialog.show(childFragmentManager, "user withdrawal")
            } else {
                toast("회원탈퇴 안내를 확인해주십시오.")
            }
        }
    }


    private fun observerData() {
        authViewModel.withDrawal.observe(viewLifecycleOwner){ state ->
            when (state) {
                is UiState.Loading -> {
                    binding.progressBar.show()
                }
                is UiState.Failure -> {
                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    binding.progressBar.hide()
                    toast("회원탈퇴가 완료되었습니다.")
                    findNavController().navigate(R.id.action_userWithdrawalFragment_to_loginFragment)
                }
            }
        }
    }
}