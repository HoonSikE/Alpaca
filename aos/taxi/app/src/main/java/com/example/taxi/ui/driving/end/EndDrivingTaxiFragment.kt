package com.example.taxi.ui.driving.end

import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentEndDrivingTaxiBinding
import com.example.taxi.di.ApplicationClass
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class EndDrivingTaxiFragment : BaseFragment<FragmentEndDrivingTaxiBinding>(R.layout.fragment_end_driving_taxi) {

    override fun init() {
        initData()
        setOnClickListeners()
    }

    private fun initData() {
        if(ApplicationClass.prefs.carImage != ""){
            Glide.with(this).load(ApplicationClass.prefs.carImage).into(binding.imageEndTaxiCar)
        }
        binding.textEndTaxi.text = ApplicationClass.prefs.carNumber
    }

    private fun setOnClickListeners(){

        binding.buttonEndTaxiPhoto.setOnClickListener {
            findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_drivingTaxiCheckFragment)
        }
        binding.buttonEndTaxiStart.setOnClickListener {
            //사진 다 넣었는지 확인하기
            //TODO : RATINGBAR 값 업데이트
            //TODO : 후불일때 처리
            findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_userHomeFragment)
        }
    }

}