package com.example.taxi.ui.login.join

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.text.Editable
import android.text.TextWatcher
import android.util.Log
import android.view.View
import androidx.core.content.ContextCompat
import androidx.core.net.toUri
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.api.KakaoAPI
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.DestinationSearch
import com.example.taxi.data.dto.user.destination.DestinationSearchDto
import com.example.taxi.databinding.FragmentJoinBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.call_taxi.setting.DestinationSearchListAdapter
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.ui.mypage.update_provider.UpdateProviderDialogFragment
import com.example.taxi.ui.mypage.update_user.UpdateAddressDialogFragment
import com.example.taxi.utils.constant.*
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory

@AndroidEntryPoint
class JoinFragment : BaseFragment<FragmentJoinBinding>(R.layout.fragment_join) {
    val authViewModel: AuthViewModel by viewModels()
//    var isEachProvider = false
    // 주소
    lateinit var addressInfo : AddressInfo
    var verificationId : String = ""
    // 사진 업로드
    var pickImageFromAlbum = 0
    var uriPhoto : Uri? = "".toUri()
    // SNS 로그인 확인
    var checkCurrentUser = false
    var check = false
    var checkHome = false

    private lateinit var destinationSearchListAdapter: DestinationSearchListAdapter
    private lateinit var destination : Destination

    private val homeSearchClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,y,place,x)
        checkHome = true
        binding.editTextJoinHomeAddress.setText(destination.addressName)
        binding.recyclerJoinHomeAddress.hide()
    }

    private val destinationSearchClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,y,place,x)
        check = true
        binding.editTextJoinCompanyAddress.setText(destination.addressName)
        binding.recyclerJoinCompanyAddress.hide()
    }

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
        binding.editTextJoinHomeAddress.addTextChangedListener( object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}

            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {
                if(binding.editTextJoinHomeAddress.text.toString() == "") {
                    return
                }
                if(checkHome){
                    checkHome = false
                }else{
                    binding.recyclerJoinHomeAddress.show()
                    searchKeyword(binding.editTextJoinHomeAddress.text.toString(), true)
                }
            }

            override fun afterTextChanged(s: Editable?) {}

        })
        binding.editTextJoinCompanyAddress.addTextChangedListener( object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}

            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {
                if(binding.editTextJoinCompanyAddress.text.toString() == "") {
                    return
                }
                if(check){
                    check = false
                }else{
                    binding.recyclerJoinCompanyAddress.show()
                    searchKeyword(binding.editTextJoinCompanyAddress.text.toString(), false)
                }
            }

            override fun afterTextChanged(s: Editable?) {}

        })
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
                    if(verificationId != "") {
                        authViewModel.ckeckPhoneAuth(
                            verificationId = verificationId,
                            code = content
                        )
                    }
                }
                dialog.show(childFragmentManager, "complete phone auth")
            }
        }
        binding.buttonJoinLogin.setOnClickListener {
            if (validation()){
//                isEachProvider = binding.switchJoinIsEachProvider.isChecked
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
//                                    if(isEachProvider)
//                                        findNavController().navigate(R.id.action_joinFragment_to_joinProviderFragment)
//                                    else
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
                    binding.buttonJoinTelAuth.setBackgroundResource(R.drawable.button_end)
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
            isEachProvider = false,
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

    private fun initSearchAdapter(list : List<DestinationSearch>, chk: Boolean){
        if(!chk){
            destinationSearchListAdapter = DestinationSearchListAdapter().apply {
                onItemClickListener = destinationSearchClickListener
            }
            binding.recyclerJoinCompanyAddress.apply {
                adapter = destinationSearchListAdapter
                layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
            }
            destinationSearchListAdapter.updateList(list)
            binding.recyclerJoinCompanyAddress.visibility = View.VISIBLE
        }else{
            destinationSearchListAdapter = DestinationSearchListAdapter().apply {
                onItemClickListener = homeSearchClickListener
            }
            binding.recyclerJoinHomeAddress.apply {
                adapter = destinationSearchListAdapter
                layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
            }
            destinationSearchListAdapter.updateList(list)
            binding.recyclerJoinHomeAddress.visibility = View.VISIBLE
        }
    }

    private fun searchKeyword(keyword: String, chk: Boolean) {
        val retrofit = Retrofit.Builder()   // Retrofit 구성
            .baseUrl(KakaoApi.BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
        val api = retrofit.create(KakaoAPI::class.java)   // 통신 인터페이스를 객체로 생성
        val call = api.getSearchKeyword(KakaoApi.API_KEY, keyword)   // 검색 조건 입력

        // API 서버에 요청
        call.enqueue(object: Callback<DestinationSearchDto> {
            override fun onResponse(
                call: Call<DestinationSearchDto>,
                response: Response<DestinationSearchDto>
            ) {
                //통신 성공 (검색 결과는 response.body()에 담겨있음)
                Log.d("Test", "Raw: ${response.raw()}")
                Log.d("Test", "Body: ${response.body()}")
                if(response.body()!!.documents != null){
                    initSearchAdapter(response.body()!!.documents, chk)
                }
            }

            override fun onFailure(call: Call<DestinationSearchDto>, t: Throwable) {
                // 통신 실패
                Log.w("MainActivity", "통신 실패: ${t.message}")
            }
        })
    }
}