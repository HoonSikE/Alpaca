package com.example.taxi.ui.mypage

import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.databinding.FragmentMyPageBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.preference.PreferenceUtil
import com.google.firebase.firestore.FirebaseFirestore
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {
    val authViewModel: AuthViewModel by viewModels()
    // 사진 불러오기
    var firestore = FirebaseFirestore.getInstance()

    override fun init() {
        initData()
        setOnClickListeners()
//        observerData()
    }

    private fun initData() {
        if(ApplicationClass.userId != null){
            firestore.collection("User").document(ApplicationClass.prefs.userSeq.toString()).get().addOnCompleteListener{ task ->
                if(task.isSuccessful){
                    var user = task.result?.toObject(User::class.java)
                    // 프로필 이미지가 없다면 기본 이미지 출력
                    if(user?.profileImage != "")
                        Glide.with(this).load(user?.profileImage).into(binding.imageMyPageProfile)
                }
            }
        }
        binding.textMyPageName.text = ApplicationClass.prefs.name + "님, 안녕하세요!"
//        binding.textMyPageCount.text = ApplicationClass.userId + "회"
//        binding.textMyPageClass.text = "Silver"
    }
    private fun setOnClickListeners(){
        binding.textUpdateUserInfo.setOnClickListener{
            findNavController().navigate(R.id.updateUserInfoFragment)
        }
        binding.textUpdatePassword.setOnClickListener{
            findNavController().navigate(R.id.updatePasswordFragment)
        }
        binding.textUpdateUserLogout.setOnClickListener{
            authViewModel.logout {
                findNavController().navigate(R.id.action_myPageFragment_to_loginFragment)
            }
        }
    }
}