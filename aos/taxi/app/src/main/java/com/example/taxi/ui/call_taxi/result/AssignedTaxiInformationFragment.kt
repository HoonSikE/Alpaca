package com.example.taxi.ui.call_taxi.result

import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentAssignedTaxiInformationBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class AssignedTaxiInformationFragment : BaseFragment<FragmentAssignedTaxiInformationBinding>(R.layout.fragment_assigned_taxi_information) {

    private val providerViewModel : ProviderViewModel by viewModels()

    override fun init() {
        //TODO : 이용한 택시 목록 추가
        //TODO : 지난 목적지 리스트에 목적지 추가
        //TODO : 자주가는 목적지에 있으면 count 증가, 없으면 추가
        //TODO : 택시 이용자 목록 추가
        initData()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        binding.textAssignedTaxiInformationNumber.text = ApplicationClass.prefs.carNumber
        binding.ratingAssignedTaxiInformationRideComfort.rating = ApplicationClass.prefs.rideComfortAverage!!
        binding.ratingAssignedTaxiInformationCleanliness.rating = ApplicationClass.prefs.cleanlinessAverage!!
        if(ApplicationClass.prefs.carImage != ""){
            Glide.with(this).load(ApplicationClass.prefs.carImage).into(binding.imageAssignedTaxiInformationCar)
        }

    }

    private fun observerData() {

    }

    private fun setOnClickListeners(){
        binding.buttonAssignedTaxiInformation.setOnClickListener{
            findNavController().navigate(R.id.action_assignedTaxiInformationFragment_to_locationTrackingTaxiFragment)
        }
    }

}