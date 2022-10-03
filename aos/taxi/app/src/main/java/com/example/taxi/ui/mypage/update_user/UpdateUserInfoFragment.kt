package com.example.taxi.ui.mypage.update_user

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.util.Log
import androidx.core.content.ContextCompat
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.databinding.FragmentUpdateUserInfoBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UpdateUserInfoFragment : BaseFragment<FragmentUpdateUserInfoBinding>(R.layout.fragment_update_user_info) {
    private val updateUserInfoViewModel : UpdateUserInfoViewModel by viewModels()

    // 사진 업로드
    var pickImageFromAlbum = 0
    var uriPhoto : Uri? = null

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }
    private fun initData(){
        if(ApplicationClass.prefs.isEachProvider == true){
            binding.imageUpdateProvider.show()
        }else{
            binding.imageUpdateProvider.hide()
        }

        val profileImage = ApplicationClass.prefs?.profileImage
        if(profileImage != "")
            Glide.with(this).load(profileImage).into(binding.imageUpdateUserImage)

        binding.textUpdateUserInfoName.text = ApplicationClass.prefs.name
        binding.textUpdateUserInfoEmail.text = ApplicationClass.userId
        binding.textUpdateUserInfoPhone.text = ApplicationClass.prefs.tel
        updateUserInfoViewModel.getAddressInfo()
    }

    private fun setOnClickListeners() {
        binding.imgJoinBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.imageUpdateProvider.setOnClickListener{
            findNavController().navigate(R.id.action_updateUserInfoFragment_to_updateProviderInfoFragment)
        }
        binding.imageUpdateUserImageButton.setOnClickListener{
            // Open Album
            var photoPickerInent = Intent(Intent.ACTION_PICK)
            photoPickerInent.type = "image/*"
            startActivityForResult(photoPickerInent, pickImageFromAlbum)
        }
        binding.imageUpdatePhoneImageButton.setOnClickListener{
            val dialog = UpdateTelDialogFragment()
            dialog.setOnOKClickedListener { content ->
                binding.textUpdateUserInfoPhone.text = content
                binding.textUpdateUserInfoPhone.setTextColor(ContextCompat.getColor(requireContext(),R.color.red))
            }
            dialog.show(childFragmentManager, "update tel")
        }
        binding.imageUpdateHomeAddressImageButton.setOnClickListener{
            val dialog = UpdateAddressDialogFragment("home")
            dialog.setOnOKClickedListener { content ->
                binding.textUpdateUserInfoHomeAddress.text = content
                binding.textUpdateUserInfoHomeAddress.setTextColor(ContextCompat.getColor(requireContext(),R.color.red))
            }
            dialog.show(childFragmentManager, "update home address")
        }
        binding.imageUpdateCompanyAddressImageButton.setOnClickListener{
            val dialog = UpdateAddressDialogFragment("company")
            dialog.setOnOKClickedListener { content ->
                binding.textUpdateUserInfoCompanyAddress.text = content
                binding.textUpdateUserInfoCompanyAddress.setTextColor(ContextCompat.getColor(requireContext(),R.color.red))
            }
            dialog.show(childFragmentManager, "update company address")
        }
        /** 윗부분은 text만 바꾸는것이고 밑에가 DB에 넣는 부분임 */
        binding.buttonUserUpdateInfo.setOnClickListener{
            // 이미지 추가
            if(uriPhoto.toString() != ""){
                val user = User("", uriPhoto.toString(), "", 0, "", ApplicationClass.prefs.userSeq.toString(), false)
                updateUserInfoViewModel.addImageUpLoad(
                    user = user
                )
            }

            // 사진 추가
            updateUserInfoViewModel.updateUserTel(
                tel = binding.textUpdateUserInfoPhone.text.toString()
            )

            // 주소정보 추가 (userSeq값 할당 후 실행)
            val addressInfo = AddressInfo(binding.textUpdateUserInfoCompanyAddress.text.toString(), binding.textUpdateUserInfoHomeAddress.text.toString())
            updateUserInfoViewModel.addAddressInfo(
                addressInfo = addressInfo
            )

            findNavController().navigate(R.id.action_updateUserInfoFragment_to_myPageFragment)
        }
    }

    // 앨범에서 사진을 선택할 시 출력
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?){
        super.onActivityResult(requestCode, resultCode, data)

        if(requestCode == pickImageFromAlbum){
            if(resultCode == Activity.RESULT_OK){
                // 앨범 사진 출력
                uriPhoto = data?.data
                binding.imageUpdateUserImage.setImageURI(uriPhoto)
            }
        }
    }

    private fun observerData(){
        updateUserInfoViewModel.addressInfo.observe(viewLifecycleOwner){ state ->
            when (state) {
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
                    binding.textUpdateUserInfoHomeAddress.text = state.data.home
                    binding.textUpdateUserInfoCompanyAddress.text = state.data.company
                }
            }
        }
    }
}