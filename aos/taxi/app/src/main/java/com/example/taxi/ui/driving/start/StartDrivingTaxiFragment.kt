package com.example.taxi.ui.driving.start

import android.annotation.SuppressLint
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentStartDrivingTaxiBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.home.user.FavoritesDialogFragment
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class StartDrivingTaxiFragment : BaseFragment<FragmentStartDrivingTaxiBinding>(R.layout.fragment_start_driving_taxi) {

    var lockState = false
    var photoState = false

    override fun init() {
        initData()
        setOnClickListeners()
    }

    private fun initData() {
        if(ApplicationClass.prefs.carImage != ""){
            Glide.with(this).load(ApplicationClass.prefs.carImage).into(binding.imageStartDrivingTaxiCar)
        }
        binding.textStartDrivingTaxi.text = ApplicationClass.prefs.carNumber
    }

    private fun setOnClickListeners(){
        binding.buttonStartDrivingTaxiLock.setOnClickListener{
            if(!lockState){
                binding.textStartDrivingTaxiDoor.text = "문이 열렸습니다!"
                binding.textStartDrivingTaxiDoor.setTextColor(resources.getColor(R.color.greenTextColor))
                lockState = !lockState
                showDialog(lockState)
            }else{
                binding.textStartDrivingTaxiDoor.text = "문이 잠겼습니다!"
                binding.textStartDrivingTaxiDoor.setTextColor(resources.getColor(R.color.redTextColor))
                lockState = !lockState
                showDialog(lockState)
            }
        }
        binding.buttonStartDrivingTaxiPhoto.setOnClickListener {
            findNavController().navigate(R.id.action_startDrivingTaxiFragment_to_DrivingTaxiCheckFragment)
        }
        binding.buttonStartDrivingTaxiStart.setOnClickListener {
            //사진 다 넣었는지 확인하기
            //TODO : DrivingTaxiFragment로 이동
        }
    }

    private fun showDialog(lockState: Boolean) {
        StartDrivingLockDialogFragment(lockState).show(childFragmentManager, "StartDrivingLockDialog")
    }

}