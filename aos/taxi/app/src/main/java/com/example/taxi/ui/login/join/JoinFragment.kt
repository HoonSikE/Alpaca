package com.example.taxi.ui.login.join

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.util.Log
import androidx.core.net.toUri
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.databinding.FragmentJoinBinding
import com.example.taxi.di.ApplicationClass
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
    // 주소
    lateinit var addressInfo : AddressInfo
    // 사진 업로드
    var pickImageFromAlbum = 0
    var uriPhoto : Uri? = null

    override fun init() {
        setOnClickListeners()
    }

    private fun setOnClickListeners(){
        binding.imageJoinUserImageButton.setOnClickListener{
            // Open Album
            var photoPickerInent = Intent(Intent.ACTION_PICK)
            photoPickerInent.type = "image/*"
            startActivityForResult(photoPickerInent, pickImageFromAlbum)
        }
        binding.buttonJoinLogin.setOnClickListener {
            if (validation()){
                isEachProvider = binding.switchJoinIsEachProvider.isChecked
                authViewModel.register(
                    email = binding.editTextJoinId.text.toString(),
                    password = binding.editTextJoinPw.text.toString(),
                    user = getUserObj()
                )
                // 주소정보 추가
                addressInfo = AddressInfo(
                    binding.editTextJoinHomeAddress.text.toString(),
                    binding.editTextJoinCompanyAddress.text.toString()
                )
                observer()
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
//                    binding.buttonJoinLogin.setText("Register")
                    binding.progressBarJoinLoading.hide()
                    state.error?.let { toast(it) }
                }
                is UiState.Success -> {
//                    binding.buttonJoinLogin.setText("Register")
                    binding.progressBarJoinLoading.hide()
                    toast(state.data)

                    authViewModel.getSession { user ->
                        if (user != null){
                            ApplicationClass.userId = user.userId
                            ApplicationClass.prefs.name = user.name
                            ApplicationClass.prefs.userSeq = user.userSeq
                            ApplicationClass.prefs.tel = user.tel
                            ApplicationClass.prefs.useCount = user.useCount
                            ApplicationClass.prefs.profileImage = user.profileImage

                            // 이미지 추가
                            authViewModel.addImageUpLoad(
                                user = user
                            )

                            // 주소정보 추가 (userSeq값 할당 후 실행)
                            authViewModel.addAddressInfo(
                                addressInfo = addressInfo
                            )
                            if(isEachProvider){
                                findNavController().navigate(R.id.action_joinFragment_to_providerHomeFragment)
                            }else{
                                findNavController().navigate(R.id.action_joinFragment_to_userHomeFragment)
                            }
                        }
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
            isEachProvider = binding.switchJoinIsEachProvider.isChecked,
            profileImage = uriPhoto.toString()
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

        if (binding.editTextJoinHomeAddress.text.isNullOrEmpty()){
            binding.editTextJoinHomeAddress.setText("")
        }

        if (binding.editTextJoinCompanyAddress.text.isNullOrEmpty()){
            binding.editTextJoinCompanyAddress.setText("")
        }
        return isValid
    }

    // 앨범에서 사진을 선택할 시 출력
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?){
        super.onActivityResult(requestCode, resultCode, data)

        if(requestCode == pickImageFromAlbum){
            if(resultCode == Activity.RESULT_OK){
                // 앨범 사진 출력
                uriPhoto = data?.data
                binding.imageJoinUserImage.setImageURI(uriPhoto)
            }
        }
    }
}