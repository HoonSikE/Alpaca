package com.example.taxi.ui.call_taxi.result

import androidx.core.os.bundleOf
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentAssignedTaxiInformationBinding
import com.example.taxi.di.ApplicationClass
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class AssignedTaxiInformationFragment : BaseFragment<FragmentAssignedTaxiInformationBinding>(R.layout.fragment_assigned_taxi_information) {

    override fun init() {
        initData()
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

    private fun setOnClickListeners(){
        binding.buttonAssignedTaxiInformation.setOnClickListener{
            findNavController().navigate(R.id.action_assignedTaxiInformationFragment_to_locationTrackingTaxiFragment)
        }
    }

}