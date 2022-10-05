package com.example.taxi.ui.login.join

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.util.Log
import androidx.core.content.ContextCompat
import androidx.core.net.toUri
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.databinding.FragmentJoinBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.ui.mypage.update_provider.UpdateProviderDialogFragment
import com.example.taxi.utils.constant.*
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class JoinFragment : BaseFragment<FragmentJoinBinding>(R.layout.fragment_join) {
    val authViewModel: AuthViewModel by viewModels()
    var isEachProvider = false
    // 주소
    lateinit var addressInfo : AddressInfo
    lateinit var verificationId : String
    // 사진 업로드
    var pickImageFromAlbum = 0
    var uriPhoto : Uri? = "".toUri()
    // SNS 로그인 확인
    var checkCurrentUser = false

    override fun init() {
        initData()
        setOnClickListeners()
        observer()
    }

    private fun initData(){
        authViewModel.getCurrentUser { currentUser ->
            // SNS 로그인 시 사용
            // 일반 회원가입일 경우 쓸필요 없음
            if(currentUser != null){
                checkCurrentUser = true
                // 프로필 사진 불러오기 및 비활성화
                Glide.with(this).load(currentUser.photoUrl).into(binding.imageJoinUserImage)
                binding.imageJoinUserImageButton.disable()
                // 이메일 불러오기
                binding.editTextJoinId.setText(currentUser.email)
                // ID, PW 비활성화
                binding.editTextJoinId.disable()
                binding.editTextJoinPw.disable()
                binding.editTextJoinPwCheck.disable()
                // 이름 불러오기/비활성화
                if(currentUser.displayName != null){
                    binding.editTextJoinName.setText(currentUser.displayName)
                    binding.editTextJoinName.disable()
                }
            }else{
                // 유저 정보가 없다면 그대로 진행
            }
        }
    }

    private fun setOnClickListeners(){
        binding.imgJoinBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.imageJoinUserImageButton.setOnClickListener{
            // Open Album
            var photoPickerInent = Intent(Intent.ACTION_PICK)
            photoPickerInent.type = "image/*"
            startActivityForResult(photoPickerInent, pickImageFromAlbum)
        }
        binding.buttonJoinTelAuth.setOnClickListener{
            var activity = requireActivity()
            if(checkPhone()){
                authViewModel.phoneAuth (
                    phoneNumber = "+82" + binding.editTextJoinTel.text.toString().replace("010", "10")+binding.editTextJoinTel2.text.toString()+binding.editTextJoinTel3.text.toString(),
                    activity = activity
                )
                toast("인증번호가 전송되었습니다. 60초 이내에 입력해주세요.")
                binding.buttonJoinTelAuth.isEnabled = false

                val dialog = PhoneAuthDialogFragment()
                // 화면 밖 터치시 종료되지 않게 하기
                dialog.isCancelable = false
                dialog.setOnOKClickedListener { content ->
                    authViewModel.ckeckPhoneAuth (
                        verificationId = verificationId,
                        code = content
                    )
                }
                dialog.show(childFragmentManager, "update home address")
            }
        }
        binding.buttonJoinLogin.setOnClickListener {
            if (validation()){
                isEachProvider = binding.switchJoinIsEachProvider.isChecked
                if(!checkCurrentUser){
                    authViewModel.register(
                        email = binding.editTextJoinId.text.toString(),
                        password = binding.editTextJoinPw.text.toString(),
                        user = getUserObj()
                    )
                }else{
                    authViewModel.snsRegister(
                        user = getUserObj()
                    )
                }
                // 주소정보 추가
                addressInfo = AddressInfo(
                    binding.editTextJoinCompanyAddress.text.toString(),
                    binding.editTextJoinHomeAddress.text.toString()
                )

                authViewModel.register.observe(viewLifecycleOwner) { state ->
                    when(state){
                        is UiState.Loading -> {
                            binding.buttonJoinLogin.setText("Loading")
                            binding.progressBarJoinLoading.show()
                        }
                        is UiState.Failure -> {
                            binding.progressBarJoinLoading.hide()
                            state.error?.let { toast(it) }
                        }
                        is UiState.Success -> {
                            binding.progressBarJoinLoading.hide()
                            toast(state.data)

                            authViewModel.getSession { user ->
                                if (user != null){
                                    ApplicationClass.userId = user.userId
                                    ApplicationClass.prefs.name = user.name
                                    ApplicationClass.prefs.userSeq = user.userSeq
                                    ApplicationClass.prefs.tel = user.tel
                                    ApplicationClass.prefs.useCount = user.useCount

                                    // 주소정보 추가 (userSeq값 할당 후 실행)
                                    authViewModel.addAddressInfo(
                                        addressInfo = addressInfo
                                    )
                                    ApplicationClass.prefs.isEachProvider = user.isEachProvider
                                    if(isEachProvider)
                                        findNavController().navigate(R.id.action_joinFragment_to_joinProviderFragment)
                                    else
                                        findNavController().navigate(R.id.action_joinFragment_to_userHomeFragment)
                                }
                            }
                        }
                        else -> {}
                    }
                }

                authViewModel.snsRegister.observe(viewLifecycleOwner) { state ->
                    when(state){
                        is UiState.Loading -> {
                            binding.buttonJoinLogin.setText("Loading")
                            binding.progressBarJoinLoading.show()
                        }
                        is UiState.Failure -> {
                            binding.progressBarJoinLoading.hide()
                            state.error?.let { toast(it) }
                        }
                        is UiState.Success -> {
                            binding.progressBarJoinLoading.hide()
                            toast(state.data)

                            authViewModel.getSession { user ->
                                if (user != null){
                                    ApplicationClass.userId = user.userId
                                    ApplicationClass.prefs.name = user.name
                                    ApplicationClass.prefs.userSeq = user.userSeq
                                    ApplicationClass.prefs.tel = user.tel
                                    ApplicationClass.prefs.useCount = user.useCount

                                    // 주소정보 추가 (userSeq값 할당 후 실행)
                                    authViewModel.addAddressInfo(
                                        addressInfo = addressInfo
                                    )
                                    ApplicationClass.prefs.isEachProvider = user.isEachProvider

                                    if(isEachProvider)
                                        findNavController().navigate(R.id.action_joinFragment_to_joinProviderFragment)
                                    else
                                        findNavController().navigate(R.id.action_joinFragment_to_userHomeFragment)
                                }
                            }
                        }
                        else -> {}
                    }
                }
            }
        }
    }

    fun observer() {
        authViewModel.phoneAuth.observe(viewLifecycleOwner) { state ->
            verificationId = state
        }
        authViewModel.ckeckPhoneAuth.observe(viewLifecycleOwner) { state ->
            when(state) {
                is UiState.Loading -> {
                    binding.progressBarJoinLoading.show()
                }
                is UiState.Failure -> {
                    binding.progressBarJoinLoading.hide()
                    state.error?.let { toast(it) }
                }
                is UiState.Success -> {
                    binding.progressBarJoinLoading.hide()
                    toast(state.data)
                    binding.editTextJoinTel.isEnabled = false
                    binding.editTextJoinTel2.isEnabled = false
                    binding.editTextJoinTel3.isEnabled = false
                    binding.buttonJoinTelAuth.text = "인증완료"
                }
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


    private fun checkPhone(): Boolean {
        if (binding.editTextJoinTel.text.isNullOrEmpty()){
            toast("핸드폰번호를 입력해 주세요.")
            return false
        }

        if (binding.editTextJoinTel2.text.isNullOrEmpty()){
            toast("핸드폰번호를 입력해 주세요.")
            return false
        }

        if (binding.editTextJoinTel3.text.isNullOrEmpty()){
            toast("핸드폰번호를 입력해 주세요.")
            return false
        }
        return true
    }
    private fun validation(): Boolean {
        var validation = true
        if (binding.editTextJoinName.text.isNullOrEmpty()){
            toast("이름을 입력해 주세요.")
            return false
        }

        if (binding.editTextJoinTel.text.isNullOrEmpty()){
            toast("핸드폰번호를 입력해 주세요.")
            return false
        }

        if (binding.editTextJoinTel2.text.isNullOrEmpty()){
            toast("핸드폰번호를 입력해 주세요.")
            return false
        }

        if (binding.editTextJoinTel3.text.isNullOrEmpty()){
            toast("핸드폰번호를 입력해 주세요.")
            return false
        }

        if (binding.editTextJoinId.text.isNullOrEmpty()){
            toast("이메일을 입력해 주세요.")
            return false
        }else{
            if (!binding.editTextJoinId.text.toString().isValidEmail()){
                toast("이메일을 양식에 맞게 입력해 주세요.")
                return false
            }
        }
        authViewModel.getCurrentUser { currentUser ->
            if(currentUser == null){
                if (binding.editTextJoinPw.text.isNullOrEmpty()){
                    toast("비밀번호를 입력해 주세요.")
                    validation =  false
                }else{
                    if (binding.editTextJoinPw.text.toString().length < 6){
                        toast("비밀번호를 6자리 이상 입력해 주세요.")
                        validation =  false
                    }
                }
                if (binding.editTextJoinPwCheck.text.isNullOrEmpty()){
                    toast("비밀번호를 입력해 주세요.")
                    validation =  false
                }else{
                    if (binding.editTextJoinPwCheck.text.toString().length < 6){
                        toast("비밀번호를 6자리 이상 입력해 주세요.")
                        validation =  false
                    }
                }

                if(binding.editTextJoinPwCheck.text.toString() != binding.editTextJoinPw.text.toString()) {
                    toast("비밀번호 확인에 실패했습니다. 다시 입력해주세요.")
                        validation =  false
                }
            }
        }
        if(!validation)
            return validation

        if (binding.editTextJoinHomeAddress.text.isNullOrEmpty()){
            binding.editTextJoinHomeAddress.setText("")
        }

        if (binding.editTextJoinCompanyAddress.text.isNullOrEmpty()){
            binding.editTextJoinCompanyAddress.setText("")
        }

        if(binding.buttonJoinTelAuth.text != "인증완료"){
            toast("휴대폰 인증을 해주세요")
            return false
        }
        return true
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