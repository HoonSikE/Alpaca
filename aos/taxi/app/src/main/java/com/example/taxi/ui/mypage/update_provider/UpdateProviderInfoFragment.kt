package com.example.taxi.ui.mypage.update_provider

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
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.address_info.AddressInfo
import com.example.taxi.databinding.FragmentUpdateProviderInfoBinding
import com.example.taxi.databinding.FragmentUpdateUserInfoBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import java.security.Provider

@AndroidEntryPoint
class UpdateProviderInfoFragment : BaseFragment<FragmentUpdateProviderInfoBinding>(R.layout.fragment_update_provider_info) {
    val providerViewModel: ProviderViewModel by viewModels()

    private lateinit var updateProviderCar : ProviderCar
    // 사진 업로드
    var pickImageFromAlbum = 0
    var uriPhoto : Uri? = null

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }
    private fun initData(){
        providerViewModel.getProvider()
    }

    private fun setOnClickListeners() {
        binding.imgJoinBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.imageUpdateCarImage.setOnClickListener{
            // Open Album
            var photoPickerInent = Intent(Intent.ACTION_PICK)
            photoPickerInent.type = "image/*"
            startActivityForResult(photoPickerInent, pickImageFromAlbum)
        }

        binding.imageUpdateCarNameButton.setOnClickListener{
            val dialog = UpdateProviderDialogFragment("carName")
            dialog.setOnOKClickedListener { content ->
                binding.textUpdateCarName.text = content
                binding.textUpdateCarName.setTextColor(ContextCompat.getColor(requireContext(),R.color.red))
            }
            dialog.show(childFragmentManager, "update home address")
        }
        binding.imageUpdateCarNumberButton.setOnClickListener{
            val dialog = UpdateProviderDialogFragment("carNumber")
            dialog.setOnOKClickedListener { content ->
                binding.textUpdateCarNumber.text = content
                binding.textUpdateCarNumber.setTextColor(ContextCompat.getColor(requireContext(),R.color.red))
            }
            dialog.show(childFragmentManager, "update company address")
        }

        /** 윗부분은 text만 바꾸는것이고 밑에가 DB에 넣는 부분임 */
        binding.buttonProvierUpdate.setOnClickListener{
            if(uriPhoto != null) {
                updateProviderCar.carImage = uriPhoto.toString()
            }
            updateProviderCar.carName = binding.textUpdateCarName.text.toString()
            updateProviderCar.carNumber = binding.textUpdateCarNumber.text.toString()

            providerViewModel.updateProvider(
                providerCar = updateProviderCar
            )

            findNavController().navigate(R.id.action_updateProviderInfoFragment_to_userHomeFragment)
        }
    }

    // 앨범에서 사진을 선택할 시 출력
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?){
        super.onActivityResult(requestCode, resultCode, data)

        if(requestCode == pickImageFromAlbum){
            if(resultCode == Activity.RESULT_OK){
                // 앨범 사진 출력
                uriPhoto = data?.data
                binding.imageUpdateCarImage.setImageURI(uriPhoto)
            }
        }
    }

    private fun observerData(){
        providerViewModel.provider.observe(viewLifecycleOwner){ state ->
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
                    val carImage = state.data.car!!.carImage
                    if(carImage != ""){
                        ApplicationClass.prefs.carImage = carImage
                        Glide.with(this).load(carImage).into(binding.imageUpdateCarImage)
                    }
                    binding.textUpdateCarName.text = state.data.car!!.carName
                    ApplicationClass.prefs.carName = binding.textUpdateCarName.text.toString()
                    binding.textUpdateCarNumber.text = state.data.car!!.carNumber
                    ApplicationClass.prefs.carNumber = binding.textUpdateCarNumber.text.toString()

                    updateProviderCar = state.data.car as ProviderCar
                }
            }
        }
    }
}