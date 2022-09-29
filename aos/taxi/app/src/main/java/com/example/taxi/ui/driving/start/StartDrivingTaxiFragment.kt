package com.example.taxi.ui.driving.start

import android.annotation.SuppressLint
import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.common.InsideCarList
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.databinding.FragmentStartDrivingTaxiBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.driving.end.EndDrivingViewModel
import com.example.taxi.ui.home.user.FavoritesDialogFragment
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
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
            findNavController().navigate(R.id.action_startDrivingTaxiFragment_to_DrivingTaxiCheckFragment, bundleOf("checkState" to true))
        }
        binding.buttonStartDrivingTaxiStart.setOnClickListener {
            findNavController().navigate(R.id.action_startDrivingTaxiFragment_to_drivingTaxiFragment)
        }
    }

    private fun showDialog(lockState: Boolean) {
        StartDrivingLockDialogFragment(lockState).show(childFragmentManager, "StartDrivingLockDialog")
    }

}