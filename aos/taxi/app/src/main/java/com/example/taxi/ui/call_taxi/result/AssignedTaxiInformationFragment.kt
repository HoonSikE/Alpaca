package com.example.taxi.ui.call_taxi.result

import androidx.core.os.bundleOf
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentAssignedTaxiInformationBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class AssignedTaxiInformationFragment : BaseFragment<FragmentAssignedTaxiInformationBinding>(R.layout.fragment_assigned_taxi_information) {

    private lateinit var taxi : Taxi

    override fun init() {
        initData()
        setOnClickListeners()
    }

    private fun initData() {
        if(arguments?.getParcelable<Taxi>("Taxi")!=null){
            taxi = arguments?.getParcelable<Taxi>("Taxi") as Taxi
            binding.textAssignedTaxiInformationNumber.text = taxi.carNumber
            binding.ratingAssignedTaxiInformationRideComfort.rating = taxi.rideComfortAverage.toFloat()
            binding.ratingAssignedTaxiInformationCleanliness.rating = taxi.cleanlinessAverage.toFloat()
            if(taxi.carImage != ""){
                Glide.with(this).load(taxi.carImage).into(binding.imageAssignedTaxiInformationCar)
            }
        }
    }

    private fun setOnClickListeners(){
        binding.buttonAssignedTaxiInformation.setOnClickListener{
            findNavController().navigate(R.id.action_assignedTaxiInformationFragment_to_locationTrackingTaxiFragment, bundleOf("Taxi" to taxi))
        }
    }

}