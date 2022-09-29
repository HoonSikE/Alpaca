package com.example.taxi.ui.login.join

import android.app.Activity
import android.content.Intent
import android.net.Uri
import androidx.core.net.toUri
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.databinding.FragmentJoinBinding
import com.example.taxi.databinding.FragmentJoinProviderBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class JoinProviderFragment : BaseFragment<FragmentJoinProviderBinding>(R.layout.fragment_join_provider) {
    val providerViewModel: ProviderViewModel by viewModels()

    // 사진 업로드
    var pickImageFromAlbum = 0
    var uriPhoto : Uri? = "".toUri()

    override fun init() {
        ApplicationClass.prefs.providerId = ApplicationClass.userId
        setOnClickListeners()
    }

    private fun setOnClickListeners(){
        binding.imageJoinProviderImageButton.setOnClickListener{
            // Open Album
            var photoPickerInent = Intent(Intent.ACTION_PICK)
            photoPickerInent.type = "image/*"
            startActivityForResult(photoPickerInent, pickImageFromAlbum)
        }
        binding.buttonJoinProvider.setOnClickListener{
            if (validation()){
                providerViewModel.addProvider(
                    provider = getProvider()
                )
                observer()
            }
        }
    }

    private fun observer(){
        providerViewModel.provider.observe(viewLifecycleOwner) { state ->
            when(state){
                is UiState.Loading -> {
                    binding.buttonJoinProvider.setText("Loading")
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

                    val provider: Provider = state.data

                    ApplicationClass.prefs.carName = provider.car?.carName
                    ApplicationClass.prefs.carNumber = provider.car?.carNumber
                    ApplicationClass.prefs.cleanlinessAverage = provider.car?.cleanlinessAverage?.toFloat()
                    ApplicationClass.prefs.isEachDriving = provider.car?.isEachDriving
                    ApplicationClass.prefs.isEachInOperation = provider.car?.isEachInOperation
                    ApplicationClass.prefs.latitude = provider.car?.position?.lati
                    ApplicationClass.prefs.longitude = provider.car?.position?.long
                    ApplicationClass.prefs.rideComfortAverage = provider.car?.rideComfortAverage?.toFloat()
                    ApplicationClass.prefs.fee = provider.revenue

                    findNavController().navigate(R.id.action_joinProviderFragment_to_providerHomeFragment)
                }
                else -> {}
            }
        }
    }

    private fun getProvider(): Provider {
        return Provider(
            driverLicense = true,
            revenue = 0,
            car = ProviderCar(
                carImage = uriPhoto.toString(),
                carName = binding.editTextJoinProviderCarName.text.toString(),
                carNumber = binding.editTextJoinProviderCarNumber.text.toString(),
                cleanlinessAverage = 0.0,
                deadLine = "",
                isEachDriving = false,
                isEachInOperation = false,
                position = Location("", ""),
                rideComfortAverage = 0.0
            )
        )
    }

    private fun validation(): Boolean {
        if(binding.editTextJoinProviderCarName.text.isNullOrEmpty()){
            toast("차량 이름을 입력해 주세요.")
            return false
        }
        if(binding.editTextJoinProviderCarNumber.text.isNullOrEmpty()){
            toast("차량 번호를 입력해 주세요.")
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
                binding.imageJoinProviderImage.setImageURI(uriPhoto)
            }
        }
    }
}